use digital_twin_glue::prelude::*;
use nalgebra::Vector3;
use uom::si::acceleration::meter_per_second_squared;
use uom::si::angle::radian;
use uom::si::angular_velocity::radian_per_second;
use uom::si::f64::{Acceleration, Angle, Force, Length, Mass, Time};
use uom::si::force::newton;
use uom::si::length::meter;
use uom::si::mass::kilogram;
use uom::si::time::second;

#[derive(Debug, Clone)]
pub struct Pid {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub integral: f64,
    pub prev_error: f64,
}

impl Pid {
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            prev_error: 0.0,
        }
    }

    pub fn update(&mut self, error: f64, dt: f64) -> f64 {
        if dt <= 0.0 {
            return 0.0;
        }

        self.integral += error * dt;
        let derivative = (error - self.prev_error) / dt;
        self.prev_error = error;

        (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
    }
}

/// Flight Computer Firmware Abstraction
#[derive(Debug, Clone)]
pub struct FlightComputer {
    pub config: MissileConfig,
    pub time: f64,
    pub pitch_pid: Pid,
    pub yaw_pid: Pid,
    pub target_waypoint: Option<Vector3<Length>>,
}

impl FlightComputer {
    pub fn new(config: MissileConfig, target_waypoint: Option<Vector3<Length>>) -> Self {
        let pitch_pid = Pid::new(
            config.pitch_pid_kp,
            config.pitch_pid_ki,
            config.pitch_pid_kd,
        );
        let yaw_pid = Pid::new(config.yaw_pid_kp, config.yaw_pid_ki, config.yaw_pid_kd);

        Self {
            config,
            time: 0.0,
            pitch_pid,
            yaw_pid,
            target_waypoint,
        }
    }

    /// Evaluates the configured motor impulse curve at the current firmware execution time
    pub fn current_thrust(&self) -> Force {
        let points = &self.config.motor_impulse_curve;
        if points.is_empty() {
            return Force::new::<newton>(0.0);
        }

        let first_t = points[0].0.get::<second>();
        let last_t = points.last().unwrap().0.get::<second>();

        if self.time <= first_t {
            return points[0].1;
        }
        if self.time >= last_t {
            return points.last().unwrap().1;
        }

        for i in 0..points.len() - 1 {
            let t0 = points[i].0.get::<second>();
            let t1 = points[i + 1].0.get::<second>();

            if self.time >= t0 && self.time <= t1 {
                let f0 = points[i].1;
                let f1 = points[i + 1].1;
                let percent = (self.time - t0) / (t1 - t0);
                return f0 + (f1 - f0) * percent;
            }
        }

        Force::new::<newton>(0.0)
    }

    pub fn tick(&mut self, state: &MissileState, dt: f64) -> MissileState {
        let mut new_state = state.clone();

        // 1. Advance the firmware cycle clock
        self.time += dt;

        // 2. Gather IMU angular velocity
        let w_pitch = state.angular_velocity[0].get::<radian_per_second>();
        let w_yaw = state.angular_velocity[1].get::<radian_per_second>();

        // 3. Command: Compute desired rates to steer towards waypoint, or default to zero
        let mut target_w_pitch = 0.0;
        let mut target_w_yaw = 0.0;

        if let Some(waypoint) = self.target_waypoint {
            let dx_m = waypoint[0].get::<meter>() - state.position[0].get::<meter>();
            let dy_m = waypoint[1].get::<meter>() - state.position[1].get::<meter>();
            let dz_m = waypoint[2].get::<meter>() - state.position[2].get::<meter>();

            let dist = (dx_m.powi(2) + dy_m.powi(2) + dz_m.powi(2)).sqrt();
            if dist > 0.1 {
                let los_world = Vector3::new(dx_m / dist, dy_m / dist, dz_m / dist);
                let los_body = state.orientation.inverse() * los_world;

                // Assuming +Z is the forward axis of the rocket nose
                let forward_body = Vector3::new(0.0, 0.0, 1.0);

                // Cross product yields the rotational axis and error magnitude needed to align
                // with the line-of-sight vector.
                let error_axis = forward_body.cross(&los_body);

                // Proportional gain converting angle error (radians) to target angular rate (rad/s)
                let k_align = 2.0;

                // X-axis rotation is pitch, Y-axis rotation is yaw.
                target_w_pitch = error_axis.x * k_align;
                target_w_yaw = error_axis.y * k_align;
            }
        }

        // 4. Compute error between IMU and intent
        let error_pitch = target_w_pitch - w_pitch;
        let error_yaw = target_w_yaw - w_yaw;

        // 5. Update PID
        let pitch_cmd = self.pitch_pid.update(error_pitch, dt);
        let yaw_cmd = self.yaw_pid.update(error_yaw, dt);

        // 6. Actuate: Command TVC/Fins servos respecting structural boundaries (e.g. +/- 20 degrees)
        let max_gimbal = 20.0_f64.to_radians();

        // We negate the commands because a positive gimbal deflection on the nozzle
        // physically creates a negative torque moment around the CG.
        let pitch_actuated = (-pitch_cmd).clamp(-max_gimbal, max_gimbal);
        let yaw_actuated = (-yaw_cmd).clamp(-max_gimbal, max_gimbal);

        new_state.tvc_angles[0] = Angle::new::<radian>(pitch_actuated);
        new_state.tvc_angles[1] = Angle::new::<radian>(yaw_actuated);

        // 7. Track Motor Propellant Mass Depletion & Store Thrust Force
        let thrust = if state.propellant_mass > Mass::new::<kilogram>(0.0) {
            self.current_thrust()
        } else {
            Force::new::<newton>(0.0) // Out of fuel! Engine flameout.
        };

        new_state.motor_thrust = thrust;

        if thrust > Force::new::<newton>(0.0) {
            // Approximation for solid motor: m_dot = F / (Isp * g0)
            let isp = Time::new::<second>(250.0);
            let g0 = Acceleration::new::<meter_per_second_squared>(9.80665);
            let m_dot = thrust / (isp * g0); // kg/sec

            let dt_time = Time::new::<second>(dt);
            let burned_mass = m_dot * dt_time;

            let mut updated_propellant = state.propellant_mass - burned_mass;
            if updated_propellant < Mass::new::<kilogram>(0.0) {
                updated_propellant = Mass::new::<kilogram>(0.0); // Never drop below 0 propellant
            }

            new_state.propellant_mass = updated_propellant;
        }

        new_state
    }
}

pub struct TheRocket {
    pub fc: FlightComputer,
}

impl TheRocket {
    pub fn new(config: MissileConfig, waypoint: Option<Vector3<Length>>) -> Self {
        Self {
            fc: FlightComputer::new(config, waypoint),
        }
    }
}

impl Rocket for TheRocket {
    fn update(&mut self, state: &MissileState, dt: f64) -> MissileState {
        // Execute the Flight Computer firmware loop mapping intention to output actuation
        self.fc.tick(state, dt)
    }
}
