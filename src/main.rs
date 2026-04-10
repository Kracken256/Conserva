use digital_twin::prelude::*;
use digital_twin_glue::prelude::*;
use nalgebra::Vector3;
use std::fs::File;
use std::io::{self, Write};
use std::time::Instant;
use uom::si::length::meter;
use uom::si::velocity::meter_per_second;

fn round_json_f64(val: &mut serde_json::Value) {
    match val {
        serde_json::Value::Number(n) => {
            if let Some(f) = n.as_f64() {
                let rounded = (f * 1e6).round() / 1e6;
                if let Some(new_n) = serde_json::Number::from_f64(rounded) {
                    *n = new_n;
                }
            }
        }
        serde_json::Value::Array(a) => {
            for v in a.iter_mut() {
                round_json_f64(v);
            }
        }
        serde_json::Value::Object(o) => {
            for v in o.values_mut() {
                round_json_f64(v);
            }
        }
        _ => {}
    }
}

fn print_state(state: &MissileState, target: Option<Vector3<uom::si::f64::Length>>) {
    let wv = state.orientation * state.body_velocity.map(|v| v.get::<meter_per_second>());

    let target_str = if let Some(t) = target {
        format!(
            "[{:.06}, {:.06}, {:.06}]",
            t[0].get::<meter>(),
            t[1].get::<meter>(),
            t[2].get::<meter>()
        )
    } else {
        "null".to_string()
    };

    println!(
        "{{\"pos\": [{:.06}, {:.06}, {:.06}], \"vel\": [{:.06}, {:.06}, {:.06}], \"ori\": [{:.06}, {:.06}, {:.06}, {:.06}], \"target\": {}}}",
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

fn save_config_json(config: &MissileConfig) {
    let config_file =
        File::create("missile.config.json").expect("Failed to create missile.config.json");
    let mut config_val = serde_json::to_value(config).expect("Failed to convert config to value");
    round_json_f64(&mut config_val);
    serde_json::to_writer_pretty(config_file, &config_val)
        .expect("Failed to write to missile.config.json");
}

fn write_state_json(flight_file: &mut File, state: &MissileState, first_json_entry: &mut bool) {
    let mut state_val = serde_json::to_value(state).expect("Failed to convert state to value");
    round_json_f64(&mut state_val);
    let json_str = serde_json::to_string(&state_val).expect("Failed to serialize state");

    if !*first_json_entry {
        writeln!(flight_file, ",").expect("Failed to write comma");
    }
    write!(flight_file, "  {}", json_str).expect("Failed to write JSON entry");
    *first_json_entry = false;
}

fn main() {
    let config = get_default_config();
    let state = get_initial_state(&config);
    let mesh_generator = TheMeshGenerator::default();

    // Dump config when the program starts
    save_config_json(&config);

    let mut flight_file =
        File::create("missile.flight.json").expect("Failed to create missile.flight.json");
    writeln!(flight_file, "[").expect("Failed to write array start");
    let mut first_json_entry = true;

    let waypoint = Some(Vector3::new(
        uom::si::f64::Length::new::<meter>(1000.0),
        uom::si::f64::Length::new::<meter>(1000.0),
        uom::si::f64::Length::new::<meter>(1000.0),
    ));
    let rocket = TheRocket::new(config.clone(), waypoint);

    let mut twin = DigitalTwin::new(config, state, Box::new(mesh_generator), Box::new(rocket));

    // Add a realistic breeze
    twin.config.environment.wind_velocity = Vector3::new(
        uom::si::f64::Velocity::new::<meter_per_second>(3.0),
        uom::si::f64::Velocity::new::<meter_per_second>(1.5),
        uom::si::f64::Velocity::new::<meter_per_second>(0.0),
    );
    // Add mild atmospheric turbulence
    twin.config.environment.turbulence_intensity =
        uom::si::f64::Velocity::new::<meter_per_second>(1.2);

    let mut last_frame_time = Instant::now();
    let mut last_print_time = Instant::now();
    let mut last_dump_time = Instant::now();

    // Slow down simulation relative to real time (e.g., 0.25x speed)
    let time_scale = 1.0;

    loop {
        // 1. Calculate how much real time has passed since the last loop iteration
        let now = Instant::now();
        let dt = now.duration_since(last_frame_time).as_secs_f64();
        last_frame_time = now;

        // 2. Step the physics engine by that exact amount
        // We limit dt to max 0.001s (1ms) per step to prevent RK4 numerical explosion from large angular rates
        // If the frame takes longer due to CPU load, the simulation will just slow down instead of accumulating NaNs.
        let sim_dt = dt.min(0.001) * time_scale;
        if sim_dt > 0.0 {
            twin.step(sim_dt);
        }

        // Check if we reached the target waypoint
        if let Some(wp) = waypoint {
            let dx = wp[0].get::<meter>() - twin.state.position[0].get::<meter>();
            let dy = wp[1].get::<meter>() - twin.state.position[1].get::<meter>();
            let dz = wp[2].get::<meter>() - twin.state.position[2].get::<meter>();
            let distance = (dx * dx + dy * dy + dz * dz).sqrt();

            // Break the simulation loop if we are within a reasonable proximity radius
            if distance < 0.2 {
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

        // 4. Dump state to flight file every 100ms
        if now.duration_since(last_dump_time).as_millis() >= 100 {
            write_state_json(&mut flight_file, &twin.state, &mut first_json_entry);
            last_dump_time = now;
        }
    }

    write_state_json(&mut flight_file, &twin.state, &mut first_json_entry);
    writeln!(flight_file, "\n]").expect("Failed to write array end");
}
