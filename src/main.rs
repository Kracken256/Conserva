use digital_twin::prelude::*;
use digital_twin_glue::prelude::*;
use nalgebra::Vector3;
use std::io::{self, Write};
use std::time::{Duration, Instant};
use uom::si::length::meter;
use uom::si::velocity::meter_per_second;

fn print_state(state: &MissileState, target: Option<Vector3<uom::si::f64::Length>>) {
    let wv = state.orientation * state.body_velocity.map(|v| v.get::<meter_per_second>());

    let target_str = if let Some(t) = target {
        format!(
            "[{}, {}, {}]",
            t[0].get::<meter>(),
            t[1].get::<meter>(),
            t[2].get::<meter>()
        )
    } else {
        "null".to_string()
    };

    println!(
        "{{\"pos\": [{}, {}, {}], \"vel\": [{}, {}, {}], \"ori\": [{}, {}, {}, {}], \"target\": {}}}",
        state.position[0].get::<meter>(),
        state.position[1].get::<meter>(),
        state.position[2].get::<meter>(),
        wv.x,
        wv.y,
        wv.z,
        state.orientation.i,
        state.orientation.j,
        state.orientation.k,
        state.orientation.w,
        target_str
    );
    let _ = io::stdout().flush();
}

fn main() {
    let state = get_initial_state();
    let config = get_default_config();
    let mesh_generator = TheMeshGenerator::default();

    let waypoint = Some(Vector3::new(
        uom::si::f64::Length::new::<meter>(1000.0),
        uom::si::f64::Length::new::<meter>(1000.0),
        uom::si::f64::Length::new::<meter>(1000.0),
    ));
    let rocket = TheRocket::new(config.clone(), waypoint);

    let mut twin = DigitalTwin::new(config, state, Box::new(mesh_generator), Box::new(rocket));

    let mut last_frame_time = Instant::now();
    let mut last_print_time = Instant::now();

    loop {
        // 1. Calculate how much real time has passed since the last loop iteration
        let now = Instant::now();
        let dt = now.duration_since(last_frame_time).as_secs_f64();
        last_frame_time = now;

        // 2. Step the physics engine by that exact amount
        // We limit dt to max 0.01s (10ms) per step to prevent RK4 numerical explosion from large angular rates
        // If the frame takes longer due to CPU load, the simulation will just slow down instead of accumulating NaNs.
        let sim_dt = dt.min(0.01);
        if sim_dt > 0.0 {
            twin.step(sim_dt);
        }

        // Check if we reached the target waypoint
        if let Some(wp) = waypoint {
            let dx = wp[0].get::<meter>() - twin.state.position[0].get::<meter>();
            let dy = wp[1].get::<meter>() - twin.state.position[1].get::<meter>();
            let dz = wp[2].get::<meter>() - twin.state.position[2].get::<meter>();
            let distance = (dx * dx + dy * dy + dz * dz).sqrt();

            // Break the simulation loop if we are within a reasonable proximity radius (e.g., 20 meters)
            if distance < 30.0 {
                println!(
                    "\n[SIMULATION STOPPED] Target hit! Final distance: {:.2} meters",
                    distance
                );
                print_state(&twin.state, waypoint);
                break;
            }
        }

        // Failsafe: stop if we hit the ground
        if twin.state.position[2].get::<meter>() <= 0.0 {
            println!("\n[SIMULATION STOPPED] Ground impact!");
            print_state(&twin.state, waypoint);
            break;
        }

        // 3. Print the state more frequently for realtime GUI validation
        if now.duration_since(last_print_time).as_millis() >= 33 {
            print_state(&twin.state, waypoint);
            last_print_time = now;
        }

        // 4. Optional: Yield to the OS to prevent 100% CPU usage
        // A very tiny sleep keeps the loop responsive without hurting physics accuracy
        std::thread::sleep(Duration::from_millis(1));
    }
}
