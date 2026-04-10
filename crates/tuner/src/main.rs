use digital_twin::prelude::*;
use digital_twin_glue::prelude::*;
use indicatif::{ProgressBar, ProgressStyle};
use nalgebra::Vector3;
use rand::Rng;
use rayon::prelude::*;
use uom::si::f64::Length;
use uom::si::length::meter;

#[derive(Clone, Debug)]
struct Wolf {
    position: (f64, f64, f64),
    score: f64,
}

fn evaluate_pid(kp: f64, ki: f64, kd: f64, max_time: f64, dt: f64) -> f64 {
    let mut config = get_default_config();
    // Apply tuned parameters to both axes for symmetry in this test
    config.controller.pitch_pid_kp = kp;
    config.controller.pitch_pid_ki = ki;
    config.controller.pitch_pid_kd = kd;
    config.controller.yaw_pid_kp = kp;
    config.controller.yaw_pid_ki = ki;
    config.controller.yaw_pid_kd = kd;

    let state = get_initial_state(&config);
    let mesh_generator = TheMeshGenerator::default();

    let target_vec = Vector3::new(1000.0, 1000.0, 1000.0);
    let waypoint = Some(target_vec.map(|c| Length::new::<meter>(c)));

    let rocket = TheRocket::new(config.clone(), waypoint);
    let mut twin = DigitalTwin::new(config, state, Box::new(mesh_generator), Box::new(rocket));

    // For evaluate tracking speed & accuracy
    let mut time = 0.0;

    let mut itae_accumulator = 0.0;
    let mut min_dist = f64::MAX;
    let mut landed_early = false;
    let mut max_w = 0.0_f64;
    let mut total_w = 0.0_f64;

    // Quick evaluate limit for optimization loops
    let early_termination_limit = max_time;

    while time < early_termination_limit {
        twin.step(dt);
        time += dt;

        let current_pos = Vector3::new(
            twin.state.position[0].get::<meter>(),
            twin.state.position[1].get::<meter>(),
            twin.state.position[2].get::<meter>(),
        );

        let error = (target_vec - current_pos).norm();
        if error < min_dist {
            min_dist = error;
        }

        // ITAE: Integral of Time-weighted Absolute Error
        // This heavily penalizes oscillations that persist into late flight
        itae_accumulator += time * error * dt;

        // Track angular rate metrics to penalize jitter, tumbling, and excessive control effort
        let w_mag = twin.state.angular_velocity.map(|v| v.value).norm();
        total_w += w_mag * dt;
        if w_mag > max_w {
            max_w = w_mag;
        }

        // Instant failure cap: if the vehicle starts tumbling violently, kill the run
        // Increase the hard cap to allow the rocket to at least attempt aggressive maneuvers.
        // The continuous penalties will handle the score scaling.
        if w_mag > 20.0 {
            return f64::MAX;
        }

        if current_pos.z <= 0.0 && time > 0.5 {
            landed_early = true;
            break;
        }

        // Success condition: within 4m of waypoint
        if error < 4.0 {
            // Factor in max wobble to ensure clean intercepts instead of spiraling into the target
            // L2 regularization: heavily penalize very large gains if smaller gains achieve the same flight profile
            let gain_penalty = (kp.powi(2) + ki.powi(2) + kd.powi(2)) * 2000.0;
            return itae_accumulator + (max_w * 5000.0) + (total_w * 500.0) + gain_penalty;
        }
    }

    // Failure: Did not reach waypoint.
    // Apply a quadratic penalty to the distance constraint to force a stronger gradient towards the target.
    let failure_penalty = if landed_early { 50000.0 } else { 10000.0 };
    let gain_penalty = (kp.powi(2) + ki.powi(2) + kd.powi(2)) * 2000.0;
    itae_accumulator
        + failure_penalty
        + min_dist.powi(2)
        + (max_w * 5000.0)
        + (total_w * 500.0)
        + gain_penalty
}

fn main() {
    let mut dt = 0.001;
    let mut epochs = 400;

    let args: Vec<String> = std::env::args().collect();
    let mut i = 1;
    while i < args.len() {
        if args[i] == "--dt" && i + 1 < args.len() {
            dt = args[i + 1]
                .parse()
                .expect("--dt must be a valid positive f64 number");
            i += 1;
        } else if args[i] == "--epochs" && i + 1 < args.len() {
            epochs = args[i + 1]
                .parse()
                .expect("--epochs must be a valid integer");
            i += 1;
        }
        i += 1;
    }

    let num_wolves = 200;

    println!(
        "Starting Grey Wolf Optimizer (GWO) with dt = {}, {} wolves for {} epochs...",
        dt, num_wolves, epochs
    );

    // Search Space Boundaries
    let bounds = [(0.0, 2.0), (0.0, 1.0), (0.0, 2.0)]; // Kp, Ki, Kd

    let mut wolves: Vec<Wolf> = (0..num_wolves)
        .map(|_| {
            let mut rng = rand::thread_rng();
            let pos = (
                rng.gen_range(bounds[0].0..bounds[0].1),
                rng.gen_range(bounds[1].0..bounds[1].1),
                0.0, // Force Kd to start at 0 for the entire initial population
            );
            Wolf {
                position: pos,
                score: f64::MAX,
            }
        })
        .collect();

    let mut alpha_pos = (0.0, 0.0, 0.0);
    let mut alpha_score = f64::MAX;

    let mut beta_pos = (0.0, 0.0, 0.0);
    let mut beta_score = f64::MAX;

    let mut delta_pos = (0.0, 0.0, 0.0);
    let mut delta_score = f64::MAX;

    let pb = ProgressBar::new((epochs * num_wolves) as u64);
    pb.set_style(
        ProgressStyle::default_bar()
            .template("[{elapsed_precise} < {eta_precise}] [{bar:40.cyan/blue}] {pos}/{len} | Alpha ITAE: {msg}")
            .unwrap(),
    );

    for epoch in 0..epochs {
        // 1. Parallel Evaluation
        let scores: Vec<f64> = wolves
            .par_iter()
            .map(|w| evaluate_pid(w.position.0, w.position.1, w.position.2, 30.0, dt))
            .collect();

        // 2. Update Alpha, Beta, Delta wolves
        for (i, score) in scores.into_iter().enumerate() {
            pb.inc(1);
            wolves[i].score = score;

            if score < alpha_score {
                delta_score = beta_score;
                delta_pos = beta_pos;
                beta_score = alpha_score;
                beta_pos = alpha_pos;
                alpha_score = score;
                alpha_pos = wolves[i].position;
                pb.set_message(format!(
                    "{:.2}, Kp: {:.6}, Ki: {:.6}, Kd: {:.6}",
                    alpha_score, alpha_pos.0, alpha_pos.1, alpha_pos.2
                ));
            } else if score < beta_score {
                delta_score = beta_score;
                delta_pos = beta_pos;
                beta_score = score;
                beta_pos = wolves[i].position;
            } else if score < delta_score {
                delta_score = score;
                delta_pos = wolves[i].position;
            }
        }

        // 3. Decrement 'a' from 2.0 to 0.0
        // hyperparameter:
        // By modifying how 'a' decreases, we affect the balance between exploration and exploitation.
        // We use a slower cubic non-linear decay to give the wolves much more time to explore
        // the space before committing to exploitation.
        let epoch_ratio = epoch as f64 / epochs as f64;
        // Using powi(3) or powi(4) keeps 'a' higher for longer, slowing the decay.
        let a = 2.0 * (1.0 - epoch_ratio.powi(4));

        for w in &mut wolves {
            let mut rng = rand::thread_rng();

            let mut update_dim = |current_val: f64, target_val: f64| -> f64 {
                let r1: f64 = rng.r#gen();
                let r2: f64 = rng.r#gen();
                let a1 = 2.0 * a * r1 - a;
                let c1 = 2.0 * r2;
                let d1 = (c1 * target_val - current_val).abs();
                target_val - a1 * d1
            };

            // Update for Alpha
            let x1 = (
                update_dim(w.position.0, alpha_pos.0),
                update_dim(w.position.1, alpha_pos.1),
                update_dim(w.position.2, alpha_pos.2),
            );

            // Update for Beta
            let x2 = (
                update_dim(w.position.0, beta_pos.0),
                update_dim(w.position.1, beta_pos.1),
                update_dim(w.position.2, beta_pos.2),
            );

            // Update for Delta
            let x3 = (
                update_dim(w.position.0, delta_pos.0),
                update_dim(w.position.1, delta_pos.1),
                update_dim(w.position.2, delta_pos.2),
            );

            // Average positions and clamp to boundaries
            w.position.0 = ((x1.0 + x2.0 + x3.0) / 3.0).clamp(bounds[0].0, bounds[0].1);
            w.position.1 = ((x1.1 + x2.1 + x3.1) / 3.0).clamp(bounds[1].0, bounds[1].1);
            w.position.2 = ((x1.2 + x2.2 + x3.2) / 3.0).clamp(bounds[2].0, bounds[2].1);
        }
    }

    pb.finish();
    println!("\n--- GWO Optimization Results ---");
    println!("Final Alpha ITAE Score: {:.4}", alpha_score);
    println!("Optimal Kp: {:.6}", alpha_pos.0);
    println!("Optimal Ki: {:.6}", alpha_pos.1);
    println!("Optimal Kd: {:.6}", alpha_pos.2);
}
