use digital_twin_glue::prelude::*;
use nalgebra::Vector3;
use uom::si::angle::radian;
use uom::si::angular_velocity::radian_per_second;
use uom::si::f64::{Angle, Length, Time};
use uom::si::length::meter;
use uom::si::time::second;
use uom::si::velocity::meter_per_second;

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

    pub fn update(&mut self, error: f64, new_kp: f64, new_ki: f64, new_kd: f64, dt: f64) -> f64 {
        if dt <= 0.0 {
            return 0.0;
        }

        if self.ki > 1e-6 && new_ki > 1e-6 && (self.ki - new_ki).abs() > 1e-4 {
            self.integral *= self.ki / new_ki;
        } else if new_ki <= 1e-6 {
            self.integral = 0.0;
        }

        self.kp = new_kp;
        self.ki = new_ki;
        self.kd = new_kd;

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
        let current_time = Time::new::<second>(0.0);
        let pitch_pid = Pid::new(
            config.controller.current_pitch_kp(current_time),
            config.controller.current_pitch_ki(current_time),
            config.controller.current_pitch_kd(current_time),
        );
        let yaw_pid = Pid::new(
            config.controller.current_yaw_kp(current_time),
            config.controller.current_yaw_ki(current_time),
            config.controller.current_yaw_kd(current_time),
        );

        Self {
            config,
            time: 0.0,
            pitch_pid,
            yaw_pid,
            target_waypoint,
        }
    }

    /// Core execution loop of the flight computer.
    /// Runs continuously every time step `dt` to process sensors, update targeting,
    /// generate PI control commands, and move the actuating servos.
    pub fn tick(&mut self, state: &MissileState, dt: f64) -> MissileState {
        let mut new_state = state.clone();
        self.time += dt;

        // 1. Gather IMU and compute intended steering rates
        let (target_w_pitch, target_w_yaw) = self.compute_target_rates(state);
        let error_pitch = target_w_pitch - state.angular_velocity[0].get::<radian_per_second>();
        let error_yaw = target_w_yaw - state.angular_velocity[1].get::<radian_per_second>();

        // 2. Schedule PI gains and update controllers
        let current_time = Time::new::<second>(self.time);

        let pitch_cmd = self.pitch_pid.update(
            error_pitch,
            self.config.controller.current_pitch_kp(current_time),
            self.config.controller.current_pitch_ki(current_time),
            self.config.controller.current_pitch_kd(current_time),
            dt,
        );

        let yaw_cmd = self.yaw_pid.update(
            error_yaw,
            self.config.controller.current_yaw_kp(current_time),
            self.config.controller.current_yaw_ki(current_time),
            self.config.controller.current_yaw_kd(current_time),
            dt,
        );

        // 3. Actuate: Command TVC/Fins servos respecting structural boundaries
        let (pitch_angle, yaw_angle) = self.actuate(state, pitch_cmd, yaw_cmd, dt);
        new_state.tvc_angles[0] = pitch_angle;
        new_state.tvc_angles[1] = yaw_angle;

        // 4. Update Engine and Mass Properties
        self.update_physics_properties(&mut new_state);

        new_state
    }

    /// High-level navigation solver.
    /// If there is an active waypoint, instructs the system how fast it needs
    /// to rotate (pitch & yaw) to point its nose at the target.
    fn compute_target_rates(&self, state: &MissileState) -> (f64, f64) {
        if let Some(waypoint) = self.target_waypoint
            && let Some(los_body) = self.calculate_los_body(state, &waypoint)
        {
            return self.compute_rates_from_los(&los_body);
        }
        (0.0, 0.0)
    }

    /// Converts a world-space waypoint target into a Line of Sight (LOS) vector
    /// mapping the target directly into the rocket's local body coordinate space.
    fn calculate_los_body(
        &self,
        state: &MissileState,
        waypoint: &Vector3<Length>,
    ) -> Option<Vector3<f64>> {
        let dx_m = waypoint[0].get::<meter>() - state.position[0].get::<meter>();
        let dy_m = waypoint[1].get::<meter>() - state.position[1].get::<meter>();
        let dz_m = waypoint[2].get::<meter>() - state.position[2].get::<meter>();

        let dist = (dx_m.powi(2) + dy_m.powi(2) + dz_m.powi(2)).sqrt();
        if dist > 0.1 {
            let target_world = Vector3::new(dx_m / dist, dy_m / dist, dz_m / dist);
            let mut target_body = state.orientation.inverse() * target_world;

            // To work in wind, we steer the velocity vector rather than just pointing the nose.
            // We measure our travel drift in body frame and command the nose proportionally in the opposite direction.
            let vel = state.body_velocity.map(|v| v.get::<meter_per_second>());
            let v_mag = vel.norm();
            if v_mag > 10.0 {
                let travel_dir = vel / v_mag;
                let drift = target_body - travel_dir;
                target_body = (target_body + drift).normalize();
            }

            Some(target_body)
        } else {
            None
        }
    }

    /// Analyzes the body-relative Line of Sight (LOS) vector and outputs
    /// the target pitch and yaw rates required to rotate toward it via cross product.
    fn compute_rates_from_los(&self, los_body: &Vector3<f64>) -> (f64, f64) {
        // Assuming +Z is the forward axis of the rocket nose
        let forward_body = Vector3::new(0.0, 0.0, 1.0);

        // Cross product yields the rotational axis and error magnitude needed to align
        // with the line-of-sight vector.
        let error_axis = forward_body.cross(los_body);

        // Proportional gain converting angle error (radians) to target angular rate (rad/s)
        let k_align = 2.0;

        // X-axis rotation is pitch, Y-axis rotation is yaw.
        (error_axis.x * k_align, error_axis.y * k_align)
    }

    /// Converts raw PI angular acceleration commands into final servo orientations.
    /// Acts as the main pipeline applying real-world hardware limits such as
    /// structural stops, actuation lag delay, and motor turn speeds.
    fn actuate(
        &self,
        state: &MissileState,
        pitch_cmd: f64,
        yaw_cmd: f64,
        dt: f64,
    ) -> (Angle, Angle) {
        let (target_pitch, target_yaw) = self.apply_physical_bounds(pitch_cmd, yaw_cmd);

        let current_pitch = state.tvc_angles[0].get::<radian>();
        let current_yaw = state.tvc_angles[1].get::<radian>();

        let (lagged_pitch, lagged_yaw) =
            self.apply_latency_filter(current_pitch, current_yaw, target_pitch, target_yaw, dt);

        let (actuated_pitch, actuated_yaw) =
            self.apply_slew_rate_limits(current_pitch, current_yaw, lagged_pitch, lagged_yaw, dt);

        (
            Angle::new::<radian>(actuated_pitch),
            Angle::new::<radian>(actuated_yaw),
        )
    }

    /// Hard limit boundary preventing TVC engine bells / Fins from rotating past structural limits.
    fn apply_physical_bounds(&self, pitch_cmd: f64, yaw_cmd: f64) -> (f64, f64) {
        let max_gimbal = self.config.engine.max_tvc_angle.get::<radian>();
        (
            (-pitch_cmd).clamp(-max_gimbal, max_gimbal),
            (-yaw_cmd).clamp(-max_gimbal, max_gimbal),
        )
    }

    /// First-Order lag (PT1) filter modeling electromechanical delays preventing servo motors
    /// from settling instantaneously. Creates an exponential smoothing curve based on `tau`.
    fn apply_latency_filter(
        &self,
        current_pitch: f64,
        current_yaw: f64,
        target_pitch: f64,
        target_yaw: f64,
        dt: f64,
    ) -> (f64, f64) {
        let tau = self.config.engine.tvc_activation_delay.get::<second>();
        let alpha = if tau > 1e-6 {
            1.0 - (-dt / tau).exp()
        } else {
            1.0
        };

        let lagged_pitch = current_pitch + (target_pitch - current_pitch) * alpha;
        let lagged_yaw = current_yaw + (target_yaw - current_yaw) * alpha;
        (lagged_pitch, lagged_yaw)
    }

    /// Rate limiter ensuring the angular velocity of the moving parts never exceeds their
    /// physical rating (e.g., servo can only move 60 degrees per second).
    fn apply_slew_rate_limits(
        &self,
        current_pitch: f64,
        current_yaw: f64,
        lagged_target_pitch: f64,
        lagged_target_yaw: f64,
        dt: f64,
    ) -> (f64, f64) {
        let max_delta = self.config.engine.tvc_slew_rate.get::<radian_per_second>() * dt;

        let pitch_actuated =
            current_pitch + (lagged_target_pitch - current_pitch).clamp(-max_delta, max_delta);
        let yaw_actuated =
            current_yaw + (lagged_target_yaw - current_yaw).clamp(-max_delta, max_delta);

        (pitch_actuated, yaw_actuated)
    }

    /// Extrapolates live physics properties (mass, inertia tracking shifting CG, thrust curve output)
    /// based on the actively elapsed flight computer time clock.
    fn update_physics_properties(&self, state: &mut MissileState) {
        let current_time = Time::new::<second>(self.time);

        // Track Motor Propellant Mass Depletion & Store Thrust Force
        state.motor_thrust = self.config.engine.current_thrust(current_time);

        // Update Vehicle Mass and Inertia Tensor from Curves
        state.current_mass = self.config.mass.current_mass(current_time);
        state.inertia_tensor = self.config.mass.current_inertia_tensor(current_time);
        state.time = current_time;
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

impl RocketCtrl for TheRocket {
    fn update(&mut self, state: &MissileState, dt: f64) -> MissileState {
        // Execute the Flight Computer firmware loop mapping intention to output actuation
        self.fc.tick(state, dt)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parameters::{get_rocket_design, get_rocket_initial_state};
    use uom::si::angle::degree;
    use uom::si::angular_velocity::degree_per_second;
    use uom::si::f64::AngularVelocity;
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
        let config = get_rocket_design();
        let state = get_rocket_initial_state(&config);

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
    fn pi_proportional_response() {
        let mut pid = Pid::new(2.0, 0.0, 0.0);
        let cmd = pid.update(5.0, 2.0, 0.0, 0.0, 0.1);
        assert!((cmd - 10.0).abs() < 1e-6, "P control failed");
    }

    #[test]
    fn pi_integral_response() {
        let mut pid = Pid::new(0.0, 2.0, 0.0);
        pid.update(5.0, 0.0, 2.0, 0.0, 0.1); // error * dt * ki = 5 * 0.1 * 2 = 1.0
        let cmd1 = pid.update(5.0, 0.0, 2.0, 0.0, 0.1); // integral becomes 1.0 + 1.0 = 2.0
        assert!((cmd1 - 2.0).abs() < 1e-6, "I control failed");
    }

    #[test]
    fn current_thrust_interpolates_correctly() {
        let mut config = get_rocket_design();
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
        let mut config = get_rocket_design();
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
        let mut config = get_rocket_design();
        let state = get_rocket_initial_state(&config);

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
        let mut config = get_rocket_design();
        let state = get_rocket_initial_state(&config);
        // Give some PI values to ensure commands are generated
        config.controller.pitch_pid_kp = vec![(Time::new::<second>(0.0), 1.0)];
        config.controller.yaw_pid_kp = vec![(Time::new::<second>(0.0), 1.0)];

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

    #[test]
    fn slew_rate_limits_tvc_command() {
        let mut config = get_rocket_design();
        // Remove latency for simple slew test
        config.engine.tvc_activation_delay = Time::new::<second>(0.0);
        let slew_rate = 10.0; // deg/s
        config.engine.tvc_slew_rate = AngularVelocity::new::<degree_per_second>(slew_rate);

        let state = get_rocket_initial_state(&config);

        config.controller.pitch_pid_kp = vec![(Time::new::<second>(0.0), 100.0)]; // ensure large command

        let waypoint = Some(Vector3::new(
            Length::new::<meter>(0.0),
            Length::new::<meter>(500.0), // Off to the side for pitch command (Y-axis)
            Length::new::<meter>(500.0),
        ));

        let mut fc = FlightComputer::new(config, waypoint);
        let dt = 0.1;

        let new_state = fc.tick(&state, dt);
        let pitch = new_state.tvc_angles[0].get::<degree>();

        // Expected max delta: 10.0 deg/s * 0.1 s = 1.0 deg
        assert!(
            (pitch.abs() - 1.0).abs() < 1e-4,
            "TVC angle should be slew rate limited to 1.0 deg, got: {}",
            pitch
        );
    }

    #[test]
    fn latency_filters_tvc_command() {
        let mut config = get_rocket_design();

        let tau = 0.5; // seconds
        config.engine.tvc_activation_delay = Time::new::<second>(tau);
        // Prevent slew rate from interfering with latency test
        config.engine.tvc_slew_rate = AngularVelocity::new::<degree_per_second>(1000.0);

        let state = get_rocket_initial_state(&config);

        config.controller.pitch_pid_kp = vec![(Time::new::<second>(0.0), 10.0)];

        let waypoint = Some(Vector3::new(
            Length::new::<meter>(0.0),
            Length::new::<meter>(500.0),
            Length::new::<meter>(500.0),
        ));

        let mut fc = FlightComputer::new(config.clone(), waypoint);
        let dt = 0.1;

        // Let's run a tick with zero tau to see what the direct PI target would be
        let mut config_no_latency = config;
        config_no_latency.engine.tvc_activation_delay = Time::new::<second>(0.0);
        let mut fc_no_latency = FlightComputer::new(config_no_latency, waypoint);
        let state_no_latency = fc_no_latency.tick(&state, dt);
        let raw_target_pitch = state_no_latency.tvc_angles[0].get::<degree>();

        // Now run the tick with latency
        let new_state = fc.tick(&state, dt);
        let lagged_pitch = new_state.tvc_angles[0].get::<degree>();

        let expected_alpha = 1.0 - (-dt / tau).exp();
        let expected_lagged_pitch = raw_target_pitch * expected_alpha;

        assert!(
            (lagged_pitch - expected_lagged_pitch).abs() < 1e-4,
            "Expected latency filtered pitch {}, got: {}",
            expected_lagged_pitch,
            lagged_pitch
        );
    }
}
