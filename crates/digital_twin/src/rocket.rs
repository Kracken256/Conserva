use digital_twin_glue::prelude::*;
use nalgebra::Vector3;
use uom::si::angle::radian;
use uom::si::angular_velocity::radian_per_second;
use uom::si::f64::{Angle, Length, Time};
use uom::si::length::meter;
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
            config.controller.pitch_pid_kp,
            config.controller.pitch_pid_ki,
            config.controller.pitch_pid_kd,
        );
        let yaw_pid = Pid::new(
            config.controller.yaw_pid_kp,
            config.controller.yaw_pid_ki,
            config.controller.yaw_pid_kd,
        );

        Self {
            config,
            time: 0.0,
            pitch_pid,
            yaw_pid,
            target_waypoint,
        }
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
        let max_gimbal = self.config.engine.max_tvc_angle.get::<radian>();

        // Find the absolute target limits before applying physical actuation rate limits
        // We negate the commands because a positive gimbal deflection on the nozzle
        // physically creates a negative torque moment around the CG.
        let target_pitch = (-pitch_cmd).clamp(-max_gimbal, max_gimbal);
        let target_yaw = (-yaw_cmd).clamp(-max_gimbal, max_gimbal);

        // Account for TVC latency (first-order lag models the mechanical settling delay)
        let current_pitch = state.tvc_angles[0].get::<radian>();
        let current_yaw = state.tvc_angles[1].get::<radian>();

        let tau = self.config.engine.tvc_activation_delay.get::<second>();
        // alpha = 1.0 - exp(-dt / tau). Using a small tau approximation avoiding div-by-zero.
        let alpha = if tau > 1e-6 {
            1.0 - (-dt / tau).exp()
        } else {
            1.0
        };

        let lagged_target_pitch = current_pitch + (target_pitch - current_pitch) * alpha;
        let lagged_target_yaw = current_yaw + (target_yaw - current_yaw) * alpha;

        // Account for TVC rotation slew rate preventing instantaneous "snaps"
        let max_delta = self.config.engine.tvc_slew_rate.get::<radian_per_second>() * dt;

        let pitch_actuated =
            current_pitch + (lagged_target_pitch - current_pitch).clamp(-max_delta, max_delta);
        let yaw_actuated =
            current_yaw + (lagged_target_yaw - current_yaw).clamp(-max_delta, max_delta);

        new_state.tvc_angles[0] = Angle::new::<radian>(pitch_actuated);
        new_state.tvc_angles[1] = Angle::new::<radian>(yaw_actuated);

        // 7. Track Motor Propellant Mass Depletion & Store Thrust Force
        let thrust = self
            .config
            .engine
            .current_thrust(Time::new::<second>(self.time));

        new_state.motor_thrust = thrust;

        // 8. Update Vehicle Mass from Mass Curve
        new_state.current_mass = self
            .config
            .mass
            .current_mass(Time::new::<second>(self.time));
        new_state.inertia_tensor = self
            .config
            .mass
            .current_inertia_tensor(Time::new::<second>(self.time));
        new_state.time = Time::new::<second>(self.time);

        new_state
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::defaults::{get_default_config, get_initial_state};
    use uom::si::f64::{Force, Mass, Time};
    use uom::si::force::newton;
    use uom::si::mass::kilogram;
    use uom::si::time::second;

    /// When the rocket is already aligned with the waypoint straight ahead
    /// along the body +Z axis, the guidance law should not command any
    /// lateral pitch/yaw rates. This locks in the convention that body +Z
    /// is the forward axis used by the controller.
    #[test]
    fn waypoint_directly_ahead_requires_no_lateral_command() {
        let config = get_default_config();
        let state = get_initial_state(&config);

        // Place the waypoint directly in front of the rocket along +Z in world
        // (rocket starts at the origin with identity orientation).
        let waypoint = Some(Vector3::new(
            Length::new::<meter>(0.0),
            Length::new::<meter>(0.0),
            Length::new::<meter>(1_000.0),
        ));

        let mut fc = FlightComputer::new(config, waypoint);

        // One small guidance tick.
        let dt = 0.01;
        let new_state = fc.tick(&state, dt);

        let pitch = new_state.tvc_angles[0].get::<radian>();
        let yaw = new_state.tvc_angles[1].get::<radian>();

        // If +Z is truly the forward axis used by the guidance law, both
        // commanded gimbal angles should be (numerically) very close to zero.
        assert!(
            pitch.abs() < 1e-6,
            "expected near-zero pitch gimbal, got {}",
            pitch
        );
        assert!(
            yaw.abs() < 1e-6,
            "expected near-zero yaw gimbal, got {}",
            yaw
        );
    }

    #[test]
    fn pid_proportional_response() {
        let mut pid = Pid::new(2.0, 0.0, 0.0);
        let cmd = pid.update(5.0, 0.1);
        assert!((cmd - 10.0).abs() < 1e-6, "P control failed");
    }

    #[test]
    fn pid_integral_response() {
        let mut pid = Pid::new(0.0, 2.0, 0.0);
        pid.update(5.0, 0.1); // error * dt * ki = 5 * 0.1 * 2 = 1.0
        let cmd1 = pid.update(5.0, 0.1); // integral becomes 1.0 + 1.0 = 2.0
        assert!((cmd1 - 2.0).abs() < 1e-6, "I control failed");
    }

    #[test]
    fn pid_derivative_response() {
        let mut pid = Pid::new(0.0, 0.0, 2.0);
        pid.update(5.0, 0.1); // prev_error = 5.0
        let cmd = pid.update(10.0, 0.1); // derivative = (10 - 5) / 0.1 = 50. kd * 50 = 100
        assert!((cmd - 100.0).abs() < 1e-6, "D control failed");
    }

    #[test]
    fn current_thrust_interpolates_correctly() {
        let mut config = get_default_config();
        config.engine.motor_impulse_curve = vec![
            (Time::new::<second>(0.0), Force::new::<newton>(100.0)),
            (Time::new::<second>(1.0), Force::new::<newton>(200.0)),
            (Time::new::<second>(2.0), Force::new::<newton>(0.0)),
        ];

        let thrust = config.engine.current_thrust(Time::new::<second>(0.5));
        assert!(
            (thrust.value - 150.0).abs() < 1e-6,
            "Thrust interpolation failed"
        );

        let thrust = config.engine.current_thrust(Time::new::<second>(1.5));
        assert!(
            (thrust.value - 100.0).abs() < 1e-6,
            "Thrust interpolation failed on descending leg"
        );
    }

    #[test]
    fn current_thrust_clamps_to_curve_bounds() {
        let mut config = get_default_config();
        config.engine.motor_impulse_curve = vec![
            (Time::new::<second>(1.0), Force::new::<newton>(100.0)),
            (Time::new::<second>(2.0), Force::new::<newton>(200.0)),
        ];

        let thrust_early = config.engine.current_thrust(Time::new::<second>(0.0));
        assert!(
            (thrust_early.value - 100.0).abs() < 1e-6,
            "Thrust should clamp to first point"
        );

        let thrust_late = config.engine.current_thrust(Time::new::<second>(3.0));
        assert!(
            (thrust_late.value - 200.0).abs() < 1e-6,
            "Thrust should clamp to last point"
        );
    }

    #[test]
    fn tick_updates_mass_from_curve() {
        let mut config = get_default_config();
        let state = get_initial_state(&config);

        config.mass.mass_curve = vec![
            (Time::new::<second>(0.0), Mass::new::<kilogram>(100.0)),
            (Time::new::<second>(10.0), Mass::new::<kilogram>(50.0)),
        ];

        let mut fc = FlightComputer::new(config, None);
        fc.time = 5.0; // Halfway according to the curve

        let dt = 0.1;

        let new_state = fc.tick(&state, dt);
        let new_mass = new_state.current_mass.value;

        // At T=5.1, mass should evaluate to 74.5 (midpoint of 100 to 50)
        assert!(
            (new_mass - 74.5).abs() < 1e-6,
            "Mass interpolation incorrect: expected 74.5, got {}",
            new_mass
        );
    }

    #[test]
    fn off_axis_waypoint_generates_steering_command() {
        let mut config = get_default_config();
        let state = get_initial_state(&config);
        // Give some PID values to ensure commands are generated
        config.controller.pitch_pid_kp = 1.0;
        config.controller.yaw_pid_kp = 1.0;

        let waypoint = Some(Vector3::new(
            Length::new::<meter>(500.0), // Off to the side in +X
            Length::new::<meter>(0.0),
            Length::new::<meter>(500.0), // Ahead in +Z
        ));

        let mut fc = FlightComputer::new(config, waypoint);
        let dt = 0.1;
        let new_state = fc.tick(&state, dt);

        let pitch = new_state.tvc_angles[0].get::<radian>();
        let yaw = new_state.tvc_angles[1].get::<radian>();

        // Waypoint is +X, so LOS vector has positive X.
        // forward_body is +Z. cross(+Z, +X) = +Y (yaw axis).
        // Thus error yaw is non-zero, pitch is zero.
        assert!(
            yaw.abs() > 0.01,
            "Expected significant yaw gimbal command for off-axis target"
        );
        assert!(
            pitch.abs() < 1e-6,
            "Pitch command should be near zero for purely X-Z deviation"
        );
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
