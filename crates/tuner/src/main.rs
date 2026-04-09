use digital_twin::prelude::*;
use digital_twin_glue::prelude::*;
use indicatif::{ProgressBar, ProgressStyle};
use nalgebra::Vector3;
use rand::Rng;
use rayon::prelude::*;
use uom::si::f64::Length;
use uom::si::length::meter;

#[derive(Clone, Debug)]
struct Particle {
    position: (f64, f64, f64),
    velocity: (f64, f64, f64),
    best_position: (f64, f64, f64),
    best_score: f64,
}

fn evaluate_pid(kp: f64, ki: f64, kd: f64, max_time: f64) -> f64 {
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

    let dt = 0.01;
    let mut time = 0.0;

    let mut itae_accumulator = 0.0;
    let mut min_dist = f64::MAX;
    let mut landed_early = false;

    while time < max_time {
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

        // Penalty for high angular rates (jitter)
        let w_mag = twin.state.angular_velocity.map(|v| v.value).norm();
        itae_accumulator += w_mag * 5.0 * dt;

        if current_pos.z <= 0.0 && time > 0.5 {
            landed_early = true;
            break;
        }

        // Success condition: within 10m of waypoint
        if error < 10.0 {
            return itae_accumulator;
        }
    }

    // Failure: Did not reach waypoint.
    // Return ITAE plus a massive penalty based on how far away we finished.
    let failure_penalty = if landed_early { 5000.0 } else { 2000.0 };
    itae_accumulator + failure_penalty + (min_dist * 10.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_evaluate_pid_returns_valid_score() {
        let score = evaluate_pid(0.1, 0.01, 0.05, 1.0);
        assert!(
            score > 0.0,
            "Score should always be positive for this fitness function"
        );
        assert!(score.is_finite(), "Score must be a finite number");
    }

    #[test]
    fn test_evaluate_pid_extreme_values() {
        // High gains might cause instability, but it should still return a finite score
        let score = evaluate_pid(100.0, 100.0, 100.0, 1.0);
        assert!(
            score.is_finite(),
            "Score must be a finite number, even with extreme PID gains"
        );
    }

    #[test]
    fn test_particle_struct() {
        let p = Particle {
            position: (0.1, 0.2, 0.3),
            velocity: (0.01, 0.02, 0.03),
            best_position: (0.1, 0.2, 0.3),
            best_score: 100.0,
        };
        assert_eq!(p.position.0, 0.1);
        assert_eq!(p.best_score, 100.0);
    }
}

fn main() {
    let epochs = 60;
    let num_particles = 120;
    let mut rng = rand::thread_rng();

    // Search Space Boundaries
    let bounds = [(0.0, 1.0), (0.0, 0.1), (0.0, 1.0)]; // Kp, Ki, Kd

    let mut particles: Vec<Particle> = (0..num_particles)
        .map(|_| {
            let pos = (
                rng.gen_range(bounds[0].0..bounds[0].1),
                rng.gen_range(bounds[1].0..bounds[1].1),
                rng.gen_range(bounds[2].0..bounds[2].1),
            );
            Particle {
                position: pos,
                velocity: (0.0, 0.0, 0.0),
                best_position: pos,
                best_score: f64::MAX,
            }
        })
        .collect();

    let mut global_best_params = particles[0].position;
    let mut global_best_score = f64::MAX;

    let mut no_improvement_count = 0;
    let mut prev_best_score = f64::MAX;

    let pb = ProgressBar::new((epochs * num_particles) as u64);
    pb.set_style(
        ProgressStyle::default_bar()
            .template("[{elapsed_precise}] [{bar:40.cyan/blue}] {pos}/{len} | Best ITAE: {msg}")
            .unwrap(),
    );

    for epoch in 0..epochs {
        // 1. Parallel Evaluation
        let scores: Vec<f64> = particles
            .par_iter()
            .map(|p| evaluate_pid(p.position.0, p.position.1, p.position.2, 30.0))
            .collect();

        // 2. Update Bests
        for (i, score) in scores.into_iter().enumerate() {
            pb.inc(1);
            if score < particles[i].best_score {
                particles[i].best_score = score;
                particles[i].best_position = particles[i].position;

                if score < global_best_score {
                    global_best_score = score;
                    global_best_params = particles[i].position;
                    pb.set_message(format!(
                        "{:.2}, Kp: {:.6}, Ki: {:.6}, Kd: {:.6}",
                        global_best_score,
                        global_best_params.0,
                        global_best_params.1,
                        global_best_params.2
                    ));
                }
            }
        }

        // Check for convergence: stop if the global best score hasn't improved by a minimum threshold over 10 epochs
        if (prev_best_score - global_best_score).abs() < 1e-4 {
            no_improvement_count += 1;
        } else {
            no_improvement_count = 0;
            prev_best_score = global_best_score;
        }

        if no_improvement_count >= 10 {
            pb.println(format!("Converged early at epoch {}!", epoch + 1));
            break;
        }

        // 3. Movement with Inertia and Velocity Clamping
        let w = 0.6; // Inertia
        let c1 = 1.4; // Cognitive
        let c2 = 1.8; // Social        // 1. Parallel Evaluation

        for p in &mut particles {
            let mut rng = rand::thread_rng();

            // Update Velocity
            p.velocity.0 = w * p.velocity.0
                + c1 * rng.r#gen::<f64>() * (p.best_position.0 - p.position.0)
                + c2 * rng.r#gen::<f64>() * (global_best_params.0 - p.position.0);
            p.velocity.1 = w * p.velocity.1
                + c1 * rng.r#gen::<f64>() * (p.best_position.1 - p.position.1)
                + c2 * rng.r#gen::<f64>() * (global_best_params.1 - p.position.1);
            p.velocity.2 = w * p.velocity.2
                + c1 * rng.r#gen::<f64>() * (p.best_position.2 - p.position.2)
                + c2 * rng.r#gen::<f64>() * (global_best_params.2 - p.position.2);

            // Clamp Velocity to 10% of range to prevent chaotic jumps
            p.velocity.0 = p.velocity.0.clamp(-0.1, 0.1);
            p.velocity.1 = p.velocity.1.clamp(-0.01, 0.01);
            p.velocity.2 = p.velocity.2.clamp(-0.1, 0.1);

            // Apply Position and Clamp to bounds
            p.position.0 = (p.position.0 + p.velocity.0).clamp(bounds[0].0, bounds[0].1);
            p.position.1 = (p.position.1 + p.velocity.1).clamp(bounds[1].0, bounds[1].1);
            p.position.2 = (p.position.2 + p.velocity.2).clamp(bounds[2].0, bounds[2].1);
        }
    }

    pb.finish();
    println!("\n--- Optimization Results ---");
    println!("Final ITAE Score: {:.4}", global_best_score);
    println!("Optimal Kp: {:.6}", global_best_params.0);
    println!("Optimal Ki: {:.6}", global_best_params.1);
    println!("Optimal Kd: {:.6}", global_best_params.2);
}
