use digital_twin::prelude::*;
use digital_twin_glue::prelude::*;
use indicatif::{ProgressBar, ProgressStyle};
use nalgebra::Vector3;
use rand::Rng;
use rayon::prelude::*;
use uom::si::length::meter;

fn evaluate_pid(kp: f64, ki: f64, kd: f64) -> f64 {
    let mut config = get_default_config();
    config.pitch_pid_kp = kp;
    config.pitch_pid_ki = ki;
    config.pitch_pid_kd = kd;
    config.yaw_pid_kp = kp;
    config.yaw_pid_ki = ki;
    config.yaw_pid_kd = kd;

    let state = get_initial_state();
    let mesh_generator = TheMeshGenerator::default();

    let waypoint = Some(Vector3::new(
        uom::si::f64::Length::new::<meter>(1000.0),
        uom::si::f64::Length::new::<meter>(1000.0),
        uom::si::f64::Length::new::<meter>(1000.0),
    ));

    let rocket = TheRocket::new(config.clone(), waypoint);
    let mut twin = DigitalTwin::new(config, state, Box::new(mesh_generator), Box::new(rocket));

    let max_time = 30.0;
    let dt = 0.01;
    let mut time = 0.0;

    let mut min_distance = f64::MAX;

    while time < max_time {
        twin.step(dt);
        time += dt;

        if let Some(wp) = waypoint {
            let dx = wp[0].get::<meter>() - twin.state.position[0].get::<meter>();
            let dy = wp[1].get::<meter>() - twin.state.position[1].get::<meter>();
            let dz = wp[2].get::<meter>() - twin.state.position[2].get::<meter>();
            let distance = (dx * dx + dy * dy + dz * dz).sqrt();

            if distance < min_distance {
                min_distance = distance;
            }

            if distance < 30.0 {
                // Return a penalty based on time to incentivize faster hits
                return time;
            }
        }

        // Failsafe ground impact
        if twin.state.position[2].get::<meter>() <= 0.0 && time > 1.0 {
            break;
        }
    }

    // Large penalty for missing, proportional to best distance achieved
    1000.0 + min_distance
}

fn main() {
    println!("Starting PID tuning...");
    let mut rng = rand::thread_rng();

    // Default params (from digital_twin/src/defaults.rs)
    let mut best_score;
    let mut best_params = (0.05, 0.0, 0.0);

    // Initial evaluate
    let score = evaluate_pid(best_params.0, best_params.1, best_params.2);
    best_score = score;
    println!(
        "Initial score (Time to hit or 1000 + dist): {:.2} for params: {:?}",
        best_score, best_params
    );

    println!("Running multi-core random jitter search...");
    let epochs = 50;
    let candidates_per_epoch = 100;

    let total_evals = (epochs * candidates_per_epoch) as u64;
    let pb = ProgressBar::new(total_evals);
    pb.set_style(
        ProgressStyle::default_bar()
            .template(
                "[{elapsed_precise}] [{bar:40.cyan/blue}] {pos}/{len} eval ({eta}) | Best: {msg}",
            )
            .unwrap()
            .progress_chars("#>-"),
    );
    pb.set_message(format!("{:.2}", best_score));

    for i in 0..epochs {
        // Generate candidates
        let mut candidates = Vec::with_capacity(candidates_per_epoch);
        for _ in 0..candidates_per_epoch {
            let kp = (best_params.0 + rng.gen_range(-0.02..0.02)).max(0.0);
            let ki = (best_params.1 + rng.gen_range(-0.005..0.005)).max(0.0);
            let kd = (best_params.2 + rng.gen_range(-0.02..0.02)).max(0.0);
            candidates.push((kp, ki, kd));
        }

        // Evaluate candidates in parallel using rayon
        let results: Vec<(f64, (f64, f64, f64))> = candidates
            .into_par_iter()
            .map(|(kp, ki, kd)| {
                let s = evaluate_pid(kp, ki, kd);
                pb.inc(1);
                (s, (kp, ki, kd))
            })
            .collect();

        // Update the best score if we found a better candidate in this epoch
        for (s, (kp, ki, kd)) in results {
            if s < best_score {
                best_score = s;
                best_params = (kp, ki, kd);
                pb.println(format!(
                    "Epoch {}: New best score: {:.2} for params: {:?}",
                    i, best_score, best_params
                ));
                pb.set_message(format!("{:.2}", best_score));
            }
        }
    }

    pb.finish_with_message("Done!");

    println!("--- Tuning Complete ---");
    println!("Best Tuned PID Parameters for Pitch and Yaw:");
    println!("Kp: {}", best_params.0);
    println!("Ki: {}", best_params.1);
    println!("Kd: {}", best_params.2);
}
