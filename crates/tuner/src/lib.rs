use digital_twin::prelude::*;
use digital_twin_glue::prelude::*;
use indicatif::{ProgressBar, ProgressStyle};
use nalgebra::Vector3;
use rand::Rng;
use rayon::prelude::*;
use uom::si::angle::radian;
use uom::si::f64::{Length, Time};
use uom::si::length::meter;
use uom::si::time::second;
use uom::si::velocity::meter_per_second;

#[derive(Clone, Debug)]
struct Wolf {
    // 3 phases * 3 gains (P, I, D) = 9 dimensions total
    gains: [f64; 9],
    score: f64,
}

const STAGES: [f64; 3] = [0.0, 5.2, 10.4];

/// Professional-grade cost function for missile GNC optimization.
fn evaluate_pid(
    base_config: &MissileConfig,
    target: Vector3<f64>,
    gains: &[f64; 9],
    max_time: f64,
    dt: f64,
) -> f64 {
    let mut config = base_config.clone();

    let mut kp_curve = Vec::new();
    let mut ki_curve = Vec::new();
    let mut kd_curve = Vec::new();

    for (i, &time) in STAGES.iter().enumerate() {
        let t = Time::new::<second>(time);
        kp_curve.push((t, gains[i * 3]));
        ki_curve.push((t, gains[i * 3 + 1]));
        kd_curve.push((t, gains[i * 3 + 2]));
    }

    // Symmetric tuning for Pitch and Yaw axes
    config.controller.pitch_pid_kp = kp_curve.clone();
    config.controller.pitch_pid_ki = ki_curve.clone();
    config.controller.pitch_pid_kd = kd_curve.clone();

    config.controller.yaw_pid_kp = kp_curve;
    config.controller.yaw_pid_ki = ki_curve;
    config.controller.yaw_pid_kd = kd_curve;

    let state = get_rocket_initial_state(&config);

    let rocket = TheRocket::new(config.clone(), Some(target.map(Length::new::<meter>)));

    let mut twin = DigitalTwin::new(
        config,
        state,
        Box::new(RocketMesh::default()),
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
        total_cost += dist * (1.0 + time * 0.5) * dt;

        // 2. Control Surface Jitter
        let mut fin_jitter = 0.0;
        for i in 0..4 {
            let angle = twin.state.fin_angles[i].get::<radian>();
            fin_jitter += (angle - prev_fin_angles[i]).abs();
            prev_fin_angles[i] = angle;
        }
        total_cost += fin_jitter * 1.0;

        // 3. Structural Load Penalty
        let w_mag = twin.state.angular_velocity.map(|v| v.value).norm();
        total_cost += (w_mag * vel.norm() * 0.005) * dt;

        if time > 2.0 && dist > 50.0 && world_vel.dot(&error_vec) < 0.0 {
            return total_cost + (min_dist.powi(2) * 100.0) + 600_000.0;
        }

        if w_mag > 15.0 || (pos.z < 0.0 && time > 0.5) {
            return total_cost + (min_dist.powi(2) * 100.0) + 500_000.0;
        }

        if dist < 4.0 {
            let v_norm = world_vel.norm().max(0.001);
            let alignment = 1.0 - (world_vel.dot(&error_vec) / (v_norm * dist)).clamp(-1.0, 1.0);
            let alignment_penalty = alignment * 10000.0;

            // Need a metric to prevent excessive gains
            let gain_reg: f64 = gains.iter().sum::<f64>() * 0.1;

            return total_cost + (time * 10.0) + alignment_penalty + gain_reg;
        }
    }

    total_cost + (min_dist.powi(2) * 50.0) + 1_000_000.0
}

fn gwo_update(target_gain: f64, current_gain: f64, a: f64, rng: &mut impl Rng) -> f64 {
    let r1: f64 = rng.r#gen();
    let r2: f64 = rng.r#gen();
    let a_vec = 2.0 * a * r1 - a;
    let c_vec = 2.0 * r2;
    let d = (c_vec * target_gain - current_gain).abs();
    target_gain - a_vec * d
}

pub fn tune_pi(base_config: &MissileConfig, target: Vector3<f64>, dt: f64, iterations: usize) {
    println!("Starting Holistic Global Tuning Optimizer with dt = {}", dt);

    let num_wolves = 100;
    // kp, ki, kd bounds per stage
    let bounds_max = [100.0, 50.0, 20.0];

    let mut population: Vec<Wolf> = (0..num_wolves)
        .map(|_| {
            let mut rng = rand::thread_rng();
            let mut gains = [0.0; 9];
            for i in 0..3 {
                gains[i * 3] = rng.gen_range(0.0..bounds_max[0]);
                gains[i * 3 + 1] = rng.gen_range(0.0..bounds_max[1]);
                gains[i * 3 + 2] = rng.gen_range(0.0..bounds_max[2]);
            }
            Wolf {
                gains,
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
        population.par_iter_mut().for_each(|wolf| {
            let cost = evaluate_pid(base_config, target, &wolf.gains, 30.0, dt);

            // 3. Gain Scheduling Continuity Penalty
            let mut continuity_penalty = 0.0;
            for i in 0..2 {
                let p_diff = (wolf.gains[(i + 1) * 3] - wolf.gains[i * 3]).abs();
                let i_diff = (wolf.gains[(i + 1) * 3 + 1] - wolf.gains[i * 3 + 1]).abs();
                let d_diff = (wolf.gains[(i + 1) * 3 + 2] - wolf.gains[i * 3 + 2]).abs();
                continuity_penalty += (p_diff * 100.0) + (i_diff * 100.0) + (d_diff * 100.0);
            }

            // 4. Robust/Stochastic Tuning (Monte Carlo) Variant
            // Running variations with different target vector offsets
            let offset_target = target + Vector3::new(10.0, 0.0, 10.0);
            let cost_var = evaluate_pid(base_config, offset_target, &wolf.gains, 30.0, dt);
            let max_cost = cost.max(cost_var);

            wolf.score = max_cost + continuity_penalty;
        });

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

        let a = 2.0 * (1.0 - (iter as f64 / iterations as f64));

        population.par_iter_mut().for_each(|wolf| {
            let mut rng = rand::thread_rng();
            let mut next_gains = [0.0; 9];

            for i in 0..9 {
                let bound_idx = i % 3;
                let x1 = gwo_update(alpha.gains[i], wolf.gains[i], a, &mut rng);
                let x2 = gwo_update(beta.gains[i], wolf.gains[i], a, &mut rng);
                let x3 = gwo_update(delta.gains[i], wolf.gains[i], a, &mut rng);
                next_gains[i] = ((x1 + x2 + x3) / 3.0).clamp(0.0, bounds_max[bound_idx]);
            }
            wolf.gains = next_gains;
        });

        pb.inc(1);
        pb.set_message(format!(
            "Cost: {:.0} | Stg0[P:{:.2} I:{:.3} D:{:.3}] Stg1[P:{:.2} I:{:.3} D:{:.3}] Stg2[P:{:.2} I:{:.3} D:{:.3}]",
            alpha.score,
            alpha.gains[0], alpha.gains[1], alpha.gains[2],
            alpha.gains[3], alpha.gains[4], alpha.gains[5],
            alpha.gains[6], alpha.gains[7], alpha.gains[8]
        ));
    }

    pb.finish();

    println!(
        "
============================================="
    );
    println!("FINAL GAIN SCHEDULING LOOKUP TABLE (Copy to design.rs)");
    println!("=============================================");

    println!("        controller: MissileControllerConfig {{");

    print!("            pitch_pid_kp: vec![");
    for i in 0..3 {
        print!(
            "(Time::new::<second>({:.1}), {:.6}), ",
            STAGES[i],
            alpha.gains[i * 3]
        );
    }
    println!("],");

    print!("            pitch_pid_ki: vec![");
    for i in 0..3 {
        print!(
            "(Time::new::<second>({:.1}), {:.6}), ",
            STAGES[i],
            alpha.gains[i * 3 + 1]
        );
    }
    println!("],");

    print!("            pitch_pid_kd: vec![");
    for i in 0..3 {
        print!(
            "(Time::new::<second>({:.1}), {:.6}), ",
            STAGES[i],
            alpha.gains[i * 3 + 2]
        );
    }
    println!("],");

    print!("            yaw_pid_kp: vec![");
    for i in 0..3 {
        print!(
            "(Time::new::<second>({:.1}), {:.6}), ",
            STAGES[i],
            alpha.gains[i * 3]
        );
    }
    println!("],");

    print!("            yaw_pid_ki: vec![");
    for i in 0..3 {
        print!(
            "(Time::new::<second>({:.1}), {:.6}), ",
            STAGES[i],
            alpha.gains[i * 3 + 1]
        );
    }
    println!("],");

    print!("            yaw_pid_kd: vec![");
    for i in 0..3 {
        print!(
            "(Time::new::<second>({:.1}), {:.6}), ",
            STAGES[i],
            alpha.gains[i * 3 + 2]
        );
    }
    println!("],");

    println!("        }},");
    println!(
        "=============================================
"
    );
}
