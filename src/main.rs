use digital_twin::prelude::*;
use digital_twin_glue::prelude::*;
use nalgebra::Vector3;
use std::time::{Duration, Instant};
use uom::si::angle::radian;
use uom::si::angular_velocity::radian_per_second;
use uom::si::length::meter;
use uom::si::mass::kilogram;
use uom::si::velocity::meter_per_second;

fn print_state(state: &MissileState) {
    println!(
        "Position: [{:.04}, {:.04}, {:.04}] m",
        state.position[0].get::<meter>(),
        state.position[1].get::<meter>(),
        state.position[2].get::<meter>()
    );
    println!(
        "Velocity: [{:.04}, {:.04}, {:.04}] m/s",
        state.body_velocity[0].get::<meter_per_second>(),
        state.body_velocity[1].get::<meter_per_second>(),
        state.body_velocity[2].get::<meter_per_second>()
    );
    println!(
        "Orientation: [{:.04}, {:.04}, {:.04}, {:.04}] quaternion",
        state.orientation.i, state.orientation.j, state.orientation.k, state.orientation.w
    );
    println!(
        "Angular Velocity: [{:.04}, {:.04}, {:.04}] rad/s",
        state.angular_velocity[0].get::<radian_per_second>(),
        state.angular_velocity[1].get::<radian_per_second>(),
        state.angular_velocity[2].get::<radian_per_second>()
    );
    println!(
        "Fin Angles: [{:0.04}, {:.04}, {:.04}, {:.04}] rad",
        state.fin_angles[0].get::<radian>(),
        state.fin_angles[1].get::<radian>(),
        state.fin_angles[2].get::<radian>(),
        state.fin_angles[3].get::<radian>()
    );
    println!(
        "TVC Angles: [{:.04}, {:.04}] rad",
        state.tvc_angles[0].get::<radian>(),
        state.tvc_angles[1].get::<radian>()
    );
    println!(
        "Total Wet Mass: {:.04} kg | Propellant: {:.04} kg | Dry Mass: {:.04} kg",
        state.total_mass().get::<kilogram>(),
        state.propellant_mass.get::<kilogram>(),
        state.dry_mass.get::<kilogram>()
    );
    println!(
        "Inertia Tensor: [[{:.04}, {:.04}, {:.04}], [{:.04}, {:.04}, {:.04}], [{:.04}, {:.04}, {:.04}]] kg*m^2",
        state.inertia_tensor[(0, 0)],
        state.inertia_tensor[(0, 1)],
        state.inertia_tensor[(0, 2)],
        state.inertia_tensor[(1, 0)],
        state.inertia_tensor[(1, 1)],
        state.inertia_tensor[(1, 2)],
        state.inertia_tensor[(2, 0)],
        state.inertia_tensor[(2, 1)],
        state.inertia_tensor[(2, 2)],
    );
}

fn main() {
    let state = get_initial_state();
    let config = get_default_config();
    let mesh_generator = TheMeshGenerator::default();

    // Give the rocket a waypoint coordinates input parameter (e.g. x: 1000m, y: 1000m, z: 0m)
    let waypoint = Some(Vector3::new(
        uom::si::f64::Length::new::<meter>(1000.0),
        uom::si::f64::Length::new::<meter>(1000.0),
        uom::si::f64::Length::new::<meter>(0.0),
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
                print_state(&twin.state);
                break;
            }
        }

        // Failsafe: stop if we hit the ground
        if twin.state.position[2].get::<meter>() <= 0.0 {
            println!("\n[SIMULATION STOPPED] Ground impact!");
            print_state(&twin.state);
            break;
        }

        // 3. Print the state only once per second
        if now.duration_since(last_print_time).as_secs() >= 1 {
            println!("\n--- 1 Second Elapsed ---");
            print_state(&twin.state);
            last_print_time = now;
        }

        // 4. Optional: Yield to the OS to prevent 100% CPU usage
        // A very tiny sleep keeps the loop responsive without hurting physics accuracy
        std::thread::sleep(Duration::from_millis(1));
    }
}
