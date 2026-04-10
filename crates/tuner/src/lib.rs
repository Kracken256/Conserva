use digital_twin::prelude::*;
use digital_twin_glue::prelude::*;
use indicatif::{ProgressBar, ProgressStyle};
use nalgebra::Vector3;
use rand::Rng;
use rayon::prelude::*;
use uom::si::angle::radian;
use uom::si::f64::Length;
use uom::si::length::meter;
use uom::si::velocity::meter_per_second;

#[derive(Clone, Debug)]
struct Wolf {
    gains: [f64; 2], // [Kp, Ki]
    score: f64,
}

/// Professional-grade cost function for missile GNC optimization.
fn evaluate_pi(
    base_config: &MissileConfig,
    target: Vector3<f64>,
    kp: f64,
    ki: f64,
    max_time: f64,
    dt: f64,
) -> f64 {
    let mut config = base_config.clone();
    // Symmetric tuning for Pitch and Yaw axes
    config.controller.pitch_pi_kp = kp;
    config.controller.pitch_pi_ki = ki;
    config.controller.yaw_pi_kp = kp;
    config.controller.yaw_pi_ki = ki;

    let state = get_initial_state(&config);

    let rocket = TheRocket::new(
        config.clone(),
        Some(target.map(|c| Length::new::<meter>(c))),
    );

    let mut twin = DigitalTwin::new(
        config,
        state,
        Box::new(TheMeshGenerator::default()),
        Box::new(rocket),
    );

    let mut time = 0.0;
    let mut total_cost = 0.0;
    let mut min_dist = f64::MAX;
    let mut prev_fin_angles = [0.0; 4];

    while time < max_time {
        twin.step(dt);
        time += dt;

        let pos = Vector3::new(
            twin.state.position[0].get::<meter>(),
            twin.state.position[1].get::<meter>(),
            twin.state.position[2].get::<meter>(),
        );
        let vel = Vector3::new(
            twin.state.body_velocity[0].get::<meter_per_second>(),
            twin.state.body_velocity[1].get::<meter_per_second>(),
            twin.state.body_velocity[2].get::<meter_per_second>(),
        );
        let world_vel = twin.state.orientation * vel;

        let error_vec = target - pos;
        let dist = error_vec.norm();
        if dist < min_dist {
            min_dist = dist;
        }

        // 1. Time-Weighted Error (ITAE)
        // Gradually increases the penalty for being far away as time goes on
        total_cost += dist * (1.0 + time * 0.5) * dt;

        // 2. Control Surface Jitter (Delta-U Penalty for servo health)
        // Keep this low so that the optimizer isn't overly afraid to steer
        let mut fin_jitter = 0.0;
        for i in 0..4 {
            let angle = twin.state.fin_angles[i].get::<radian>();
            fin_jitter += (angle - prev_fin_angles[i]).abs();
            prev_fin_angles[i] = angle;
        }
        total_cost += fin_jitter * 1.0;

        // 3. Structural Load Penalty (Wobble @ Speed)
        let w_mag = twin.state.angular_velocity.map(|v| v.value).norm();
        total_cost += (w_mag * vel.norm() * 0.005) * dt;

        // If we missed the target and are moving away, terminate early to save CPU
        // We only start applying this after 2.0 seconds to give the launch thrust time to take over,
        // using the correct world-frame velocity.
        if time > 2.0 && dist > 50.0 && world_vel.dot(&error_vec) < 0.0 {
            // Missile is flying away from the target. Fast-fail it.
            return total_cost + (min_dist.powi(2) * 100.0) + 600_000.0;
        }

        // 4. Critical Failure Constraints
        if w_mag > 15.0 || (pos.z < 0.0 && time > 0.5) {
            // DO NOT return 1e12! That creates a flat plateau and destroys all gradient information.
            // Instead, heavily penalize the failure but still reward runs that got closer to the target first.
            return total_cost + (min_dist.powi(2) * 100.0) + 500_000.0;
        }

        // 5. Success Condition
        if dist < 4.0 {
            let v_norm = world_vel.norm().max(0.001);
            let alignment = 1.0 - (world_vel.dot(&error_vec) / (v_norm * dist)).clamp(-1.0, 1.0);
            let alignment_penalty = alignment * 10000.0;
            // Tiny regularization just to prevent P and I from wandering around unnecessarily when successful
            let gain_reg = (kp * 1.0) + (ki * 0.1);
            return total_cost + (time * 10.0) + alignment_penalty + gain_reg;
        }
    }

    // Failure: Did not reach target within time limit.
    // Ensure failure continues to provide strong distance-focused gradient.
    total_cost + (min_dist.powi(2) * 50.0) + 1_000_000.0
}

/// Standard GWO position update logic
fn gwo_update(target_gain: f64, current_gain: f64, a: f64, rng: &mut impl Rng) -> f64 {
    let r1: f64 = rng.r#gen();
    let r2: f64 = rng.r#gen();

    let a_vec = 2.0 * a * r1 - a;
    let c_vec = 2.0 * r2;

    let d = (c_vec * target_gain - current_gain).abs();
    target_gain - a_vec * d
}

pub fn tune_pi(base_config: &MissileConfig, target: Vector3<f64>, dt: f64, iterations: usize) {
    let num_wolves = 50;
    let bounds_max = [1.0, 1.0]; // Kp, Ki limits

    println!(
        "Starting Grey Wolf Optimizer (GWO) for PI Tuning with dt = {}, {} wolves for {} epochs...",
        dt, num_wolves, iterations
    );

    // Initialize population randomly
    let mut population: Vec<Wolf> = (0..num_wolves)
        .map(|_| {
            let mut rng = rand::thread_rng();
            Wolf {
                gains: [
                    rng.gen_range(0.0..bounds_max[0]),
                    rng.gen_range(0.0..bounds_max[1]),
                ],
                score: f64::MAX,
            }
        })
        .collect();

    let mut alpha = population[0].clone();
    let mut beta = population[0].clone();
    let mut delta = population[0].clone();

    let pb = ProgressBar::new(iterations as u64);
    pb.set_style(
        ProgressStyle::default_bar()
            .template(
                "[{elapsed_precise} < {eta_precise}] [{bar:40.cyan/blue}] {pos}/{len} | {msg}",
            )
            .unwrap(),
    );

    for iter in 0..iterations {
        // Parallel Fitness Evaluation
        population.par_iter_mut().for_each(|wolf| {
            wolf.score = evaluate_pi(base_config, target, wolf.gains[0], wolf.gains[1], 30.0, dt);
        });

        // Update Alpha, Beta, Delta (the three best wolves)
        population.sort_by(|a, b| a.score.partial_cmp(&b.score).unwrap());

        if population[0].score < alpha.score {
            alpha = population[0].clone();
        }
        if population[1].score < beta.score {
            beta = population[1].clone();
        }
        if population[2].score < delta.score {
            delta = population[2].clone();
        }

        // Linearly decreasing parameter 'a' from 2 to 0
        let a = 2.0 * (1.0 - (iter as f64 / iterations as f64));

        // Update positions based on leaders
        population.par_iter_mut().for_each(|wolf| {
            let mut rng = rand::thread_rng();
            let mut next_gains = [0.0; 2];

            for i in 0..2 {
                let x1 = gwo_update(alpha.gains[i], wolf.gains[i], a, &mut rng);
                let x2 = gwo_update(beta.gains[i], wolf.gains[i], a, &mut rng);
                let x3 = gwo_update(delta.gains[i], wolf.gains[i], a, &mut rng);

                next_gains[i] = ((x1 + x2 + x3) / 3.0).clamp(0.0, bounds_max[i]);
            }
            wolf.gains = next_gains;
        });

        pb.inc(1);
        pb.set_message(format!(
            "Score: {:.2} | [P:{:.6} I:{:.6}]",
            alpha.score, alpha.gains[0], alpha.gains[1]
        ));
    }

    pb.finish();
    println!("\n--- Optimization Complete ---");
    println!(
        "Final Optimal Gains: Kp={:.6}, Ki={:.6}",
        alpha.gains[0], alpha.gains[1]
    );
}
