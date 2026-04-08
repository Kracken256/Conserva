use std::time::{Duration, Instant};

use digital_twin::prelude::*;
use digital_twin_glue::prelude::*;

fn main() {
    let state = get_initial_state();
    let config = get_default_config();
    let solver = TheSolver::default();
    let mesh_generator = TheMeshGenerator::default();
    let flight_computer = TheFlightComputer::default();

    let mut twin = DigitalTwin::new(
        config,
        state,
        Box::new(solver),
        Box::new(mesh_generator),
        Box::new(flight_computer),
    );

    let mut last_frame_time = Instant::now();
    let mut last_print_time = Instant::now();

    loop {
        // 1. Calculate how much real time has passed since the last loop iteration
        let now = Instant::now();
        let dt = now.duration_since(last_frame_time).as_secs_f32();
        last_frame_time = now;

        // 2. Step the physics engine by that exact amount
        // We check dt > 0 to avoid potential panics or zero-division in the solver
        if dt > 0.0 {
            twin.step(dt);
        }

        // 3. Print the state only once per second
        if now.duration_since(last_print_time).as_secs() >= 1 {
            println!("--- 1 Second Elapsed ---");
            println!("Current State: {:?}", twin.state);
            last_print_time = now;
        }

        // 4. Optional: Yield to the OS to prevent 100% CPU usage
        // A very tiny sleep keeps the loop responsive without hurting physics accuracy
        std::thread::sleep(Duration::from_millis(1));
    }
}
