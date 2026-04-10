use crate::control::state::MissileState;
use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3};
use uom::si::angular_velocity::radian_per_second;
use uom::si::f64::{AngularVelocity, Length, Velocity};
use uom::si::length::meter;
use uom::si::velocity::meter_per_second;

pub struct Rk4 {}

impl Rk4 {
    #[inline]
    fn compute_derivatives<F>(
        v: Vector3<f64>,
        w: Vector3<f64>,
        q: Quaternion<f64>,
        p: Vector3<f64>,
        mass: f64,
        inertia: &Matrix3<f64>,
        inertia_inv: &Matrix3<f64>,
        mut get_forces: F,
    ) -> (Vector3<f64>, Vector3<f64>, Vector3<f64>, Quaternion<f64>)
    where
        F: FnMut(
            Vector3<f64>,
            Vector3<f64>,
            Quaternion<f64>,
            Vector3<f64>,
        ) -> (Vector3<f64>, Vector3<f64>),
    {
        let q_u = UnitQuaternion::new_normalize(q);

        // 1. SAMPLE PHYSICS: Get forces for this specific intermediate state
        let (force_body, torque_body) = get_forces(v, w, q, p);

        // 2. KINEMATICS
        let dp = q_u * v;

        // 3. LINEAR DYNAMICS (Body Frame)
        let gravity_world = Vector3::new(0.0, 0.0, -9.81);
        let gravity_body = q_u.conjugate() * gravity_world;
        let dv = (force_body / mass) + gravity_body - w.cross(&v);

        // 4. ROTATIONAL DYNAMICS (Body Frame)
        let dw = inertia_inv * (torque_body - w.cross(&(inertia * w)));

        // 5. ORIENTATION DYNAMICS (Body Frame angular velocity implies post-multiplication)
        let dq = q * Quaternion::new(0.0, w.x, w.y, w.z) * 0.5;

        (dp, dv, dw, dq)
    }

    pub fn step<F>(&mut self, state: &MissileState, mut get_forces: F, dt: f64) -> MissileState
    where
        F: FnMut(
            Vector3<f64>,
            Vector3<f64>,
            Quaternion<f64>,
            Vector3<f64>,
        ) -> (Vector3<f64>, Vector3<f64>),
    {
        let mass = state.current_mass.value;
        // Approximation: scale the identity inertia tensor by the total mass
        // For a more accurate reading, you would want to calculate the actual moment of inertia for an elongating cylinder payload vs mass distribution over time.
        let inertia = state.inertia_tensor * mass;
        let inertia_inv = inertia.try_inverse().unwrap_or_else(Matrix3::identity);

        let p = state.position.map(|c| c.value);
        let v = state.body_velocity.map(|c| c.value);
        let w = state.angular_velocity.map(|c| c.value);
        let q = *state.orientation.as_ref();

        let dt_half = dt * 0.5;

        // k1
        let (k1_p, k1_v, k1_w, k1_q) =
            Self::compute_derivatives(v, w, q, p, mass, &inertia, &inertia_inv, &mut get_forces);

        // k2
        let (k2_p, k2_v, k2_w, k2_q) = Self::compute_derivatives(
            v + k1_v * dt_half,
            w + k1_w * dt_half,
            q + k1_q * dt_half,
            p + k1_p * dt_half,
            mass,
            &inertia,
            &inertia_inv,
            &mut get_forces,
        );

        // k3
        let (k3_p, k3_v, k3_w, k3_q) = Self::compute_derivatives(
            v + k2_v * dt_half,
            w + k2_w * dt_half,
            q + k2_q * dt_half,
            p + k2_p * dt_half,
            mass,
            &inertia,
            &inertia_inv,
            &mut get_forces,
        );

        // k4
        let (k4_p, k4_v, k4_w, k4_q) = Self::compute_derivatives(
            v + k3_v * dt,
            w + k3_w * dt,
            q + k3_q * dt,
            p + k3_p * dt,
            mass,
            &inertia,
            &inertia_inv,
            &mut get_forces,
        );

        let dt_sixth = dt / 6.0;
        let next_p = p + (k1_p + k2_p * 2.0 + k3_p * 2.0 + k4_p) * dt_sixth;
        let next_v = v + (k1_v + k2_v * 2.0 + k3_v * 2.0 + k4_v) * dt_sixth;
        let next_w = w + (k1_w + k2_w * 2.0 + k3_w * 2.0 + k4_w) * dt_sixth;
        let next_q = q + (k1_q + k2_q * 2.0 + k3_q * 2.0 + k4_q) * dt_sixth;

        MissileState {
            position: next_p.map(Length::new::<meter>),
            body_velocity: next_v.map(Velocity::new::<meter_per_second>),
            orientation: UnitQuaternion::new_normalize(next_q),
            angular_velocity: next_w.map(AngularVelocity::new::<radian_per_second>),
            ..state.clone()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3};
    use uom::si::angle::radian;
    use uom::si::angular_velocity::radian_per_second;
    use uom::si::f64::*;
    use uom::si::force::newton;
    use uom::si::length::meter;
    use uom::si::mass::kilogram;
    use uom::si::time::second;
    use uom::si::velocity::meter_per_second;

    fn dummy_state() -> MissileState {
        MissileState {
            time: Time::new::<second>(0.0),
            position: Vector3::zeros().map(Length::new::<meter>),
            body_velocity: Vector3::zeros().map(Velocity::new::<meter_per_second>),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros().map(AngularVelocity::new::<radian_per_second>),
            fin_angles: [Angle::new::<radian>(0.0); 4],
            tvc_angles: [Angle::new::<radian>(0.0); 2],
            current_mass: Mass::new::<kilogram>(1.0),
            motor_thrust: Force::new::<newton>(0.0),
            inertia_tensor: Matrix3::identity(),
        }
    }

    /// Verify that the integrator interprets the body frame and world frame
    /// consistently with the documented conventions:
    /// - World Z is "up" (gravity points toward -Z in world and body when q = I).
    /// - Body +Z is the rocket's forward axis, so dp = q * v maps body +Z to world +Z.
    #[test]
    fn body_and_world_axes_conventions_are_consistent() {
        // Body-frame linear velocity: 1 m/s along body +Z (rocket nose).
        let v_body = Vector3::new(0.0, 0.0, 1.0);
        let w = Vector3::new(0.0, 0.0, 0.0);
        let p = Vector3::new(0.0, 0.0, 0.0);
        let mass = 1.0;
        let inertia = Matrix3::identity();
        let inertia_inv = Matrix3::identity();
        let q = Quaternion::identity();

        // No aerodynamic/body forces or torques; only gravity should appear in dv.
        let mut get_forces =
            |_v: Vector3<f64>, _w: Vector3<f64>, _q: Quaternion<f64>, _p: Vector3<f64>| {
                (Vector3::zeros(), Vector3::zeros())
            };

        let (dp, dv, _dw, _dq) = Rk4::compute_derivatives(
            v_body,
            w,
            q,
            p,
            mass,
            &inertia,
            &inertia_inv,
            &mut get_forces,
        );

        // With q = I, body +Z should map directly to world +Z.
        assert!((dp - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-12);

        // With q = I, gravity_world = (0,0,-9.81) should appear unchanged in body frame.
        assert!((dv - Vector3::new(0.0, 0.0, -9.81)).norm() < 1e-12);
    }

    #[test]
    fn freefall_from_rest() {
        let v = Vector3::zeros();
        let w = Vector3::zeros();
        let p = Vector3::zeros();
        let q = Quaternion::identity();
        let mass = 1.0;
        let inertia = Matrix3::identity();
        let inertia_inv = Matrix3::identity();

        let mut get_forces = |_, _, _, _| (Vector3::zeros(), Vector3::zeros());

        let (dp, dv, dw, dq) =
            Rk4::compute_derivatives(v, w, q, p, mass, &inertia, &inertia_inv, &mut get_forces);

        assert_eq!(
            dp,
            Vector3::zeros(),
            "No initial velocity means no initial position change"
        );
        assert_eq!(
            dv,
            Vector3::new(0.0, 0.0, -9.81),
            "Only gravity accelerates the body"
        );
        assert_eq!(
            dw,
            Vector3::zeros(),
            "No torque means no angular acceleration"
        );
        assert_eq!(
            dq,
            Quaternion::new(0.0, 0.0, 0.0, 0.0),
            "No angular velocity means no rotation derivative"
        );
    }

    #[test]
    fn constant_velocity_with_gravity_balanced() {
        let v = Vector3::new(10.0, 0.0, 0.0); // Moving forward purely at 10m/s
        let w = Vector3::zeros();
        let p = Vector3::zeros();
        let q = Quaternion::identity();
        let mass = 2.0;
        let inertia = Matrix3::identity();
        let inertia_inv = Matrix3::identity();

        // Upward lift forces balancing out gravity completely -> 9.81 * 2.0 = 19.62 upward
        let mut get_forces = |_, _, _, _| (Vector3::new(0.0, 0.0, 19.62), Vector3::zeros());

        let (dp, dv, _, _) =
            Rk4::compute_derivatives(v, w, q, p, mass, &inertia, &inertia_inv, &mut get_forces);

        assert_eq!(
            dp,
            Vector3::new(10.0, 0.0, 0.0),
            "World position moves straight along X"
        );
        assert_eq!(
            dv,
            Vector3::zeros(),
            "Gravity is balanced, so zero acceleration"
        );
    }

    #[test]
    fn centripetal_acceleration_kinematics() {
        let v = Vector3::new(0.0, 0.0, 10.0); // Flying forward at 10m/s
        let w = Vector3::new(2.0, 0.0, 0.0); // Pitching up at 2.0 rad/s
        let p = Vector3::zeros();
        let q = Quaternion::identity();
        let mass = 1.0;
        let inertia = Matrix3::identity();
        let inertia_inv = Matrix3::identity();

        let mut get_forces = |_, _, _, _| (Vector3::zeros(), Vector3::zeros());

        let (_, dv, _, _) =
            Rk4::compute_derivatives(v, w, q, p, mass, &inertia, &inertia_inv, &mut get_forces);

        // dv = g - w x v = (0, 0, -9.81) - ((2, 0, 0) x (0, 0, 10))
        // (2, 0, 0) x (0, 0, 10) = (0, -20, 0)
        // dv = (0, 0, -9.81) - (0, -20, 0) = (0, 20, -9.81)
        assert!(
            (dv - Vector3::new(0.0, 20.0, -9.81)).norm() < 1e-12,
            "Cross product coriolis effect matches. Got: {:?}",
            dv
        );
    }

    #[test]
    fn gyroscopic_coupling_euler_equation() {
        let v = Vector3::zeros();
        let w = Vector3::new(1.0, 1.0, 1.0); // Spinning on all axes
        let p = Vector3::zeros();
        let q = Quaternion::identity();
        let mass = 1.0;

        let inertia = Matrix3::from_diagonal(&Vector3::new(1.0, 2.0, 3.0));
        let inertia_inv = Matrix3::from_diagonal(&Vector3::new(1.0, 1.0 / 2.0, 1.0 / 3.0));

        let mut get_forces = |_, _, _, _| (Vector3::zeros(), Vector3::zeros());

        let (_, _, dw, _) =
            Rk4::compute_derivatives(v, w, q, p, mass, &inertia, &inertia_inv, &mut get_forces);

        // I = diag(1, 2, 3), w = (1, 1, 1)
        // I*w = (1, 2, 3)
        // w x (I*w) = (1, 1, 1) x (1, 2, 3) = (3-2, 1-3, 2-1) = (1, -2, 1)
        // dw = - I^-1 * (w x Iw) = - diag(1, 1/2, 1/3) * (1, -2, 1) = (-1.0, 1.0, -0.333...)
        assert!((dw.x - (-1.0)).abs() < 1e-12);
        assert!((dw.y - (1.0)).abs() < 1e-12);
        assert!((dw.z - (-1.0 / 3.0)).abs() < 1e-12);
    }

    #[test]
    fn quaternion_derivative_pure_rotation() {
        let v = Vector3::zeros();
        let w = Vector3::new(2.0, 0.0, 0.0); // 2 rad/s pitch
        let p = Vector3::zeros();
        let q = Quaternion::identity();
        let mass = 1.0;
        let inertia = Matrix3::identity();
        let inertia_inv = Matrix3::identity();

        let mut get_forces = |_, _, _, _| (Vector3::zeros(), Vector3::zeros());

        let (_, _, _, dq) =
            Rk4::compute_derivatives(v, w, q, p, mass, &inertia, &inertia_inv, &mut get_forces);

        // dq = 0.5 * q * (0, w)
        // q = I. (0, w) = (0, 2, 0, 0)
        // dq = 0.5 * (0, 2, 0, 0) = (0, 1, 0, 0)
        assert_eq!(
            dq,
            Quaternion::new(0.0, 1.0, 0.0, 0.0),
            "Quaternion derivative vector isolated property"
        );
    }

    #[test]
    fn applied_body_torque_creates_angular_acceleration() {
        let v = Vector3::zeros();
        let w = Vector3::zeros();
        let p = Vector3::zeros();
        let q = Quaternion::identity();
        let mass = 1.0;
        let inertia = Matrix3::from_diagonal(&Vector3::new(2.0, 2.0, 2.0));
        let inertia_inv = Matrix3::from_diagonal(&Vector3::new(0.5, 0.5, 0.5));

        // Applied pure torque on the body x axis
        let mut get_forces = |_, _, _, _| (Vector3::zeros(), Vector3::new(10.0, 0.0, 0.0));

        let (_, _, dw, _) =
            Rk4::compute_derivatives(v, w, q, p, mass, &inertia, &inertia_inv, &mut get_forces);

        // dw = I_inv * Torque = 0.5 * 10 = 5
        assert_eq!(dw, Vector3::new(5.0, 0.0, 0.0));
    }

    #[test]
    fn get_forces_called_exactly_four_times() {
        let mut rk4 = Rk4 {};
        let state = dummy_state();
        let mut call_count = 0;

        rk4.step(
            &state,
            |_, _, _, _| {
                call_count += 1;
                (Vector3::zeros(), Vector3::zeros())
            },
            0.1,
        );

        assert_eq!(
            call_count, 4,
            "RK4 is required to evaluate intermediate states exactly 4 times"
        );
    }

    #[test]
    fn integration_step_updates_gravity_freefall_exactly() {
        let mut rk4 = Rk4 {};
        let state = dummy_state();

        let next_state = rk4.step(
            &state,
            |_, _, _, _| (Vector3::zeros(), Vector3::zeros()),
            1.0,
        );

        // Gravity applies a constant acceleration of -9.81 m/s^2 perfectly matching RK4
        assert!(
            (next_state.body_velocity.z.value - (-9.81)).abs() < 1e-12,
            "Final velocity equals g*dt"
        );
        // Integral of an integrated linear function provides exactly dt^2 / 2 for position
        assert!(
            (next_state.position.z.value - (-4.905)).abs() < 1e-12,
            "Final position equals 0.5*g*dt^2"
        );
    }

    #[test]
    fn non_singular_inertia_handled_gracefully() {
        let mut rk4 = Rk4 {};
        let mut state = dummy_state();
        // Give an uninvertible inertia (0 matrix)
        state.inertia_tensor = Matrix3::zeros();

        // Torques apply natively over identity fallback for the uninvertible tensor
        let mut call_count = 0;
        rk4.step(
            &state,
            |_, _, _, _| {
                call_count += 1;
                (Vector3::zeros(), Vector3::new(3.0, 3.0, 3.0))
            },
            1.0,
        );

        // Should fall back to identity matrix inverse successfully inside step!
        assert_eq!(
            call_count, 4,
            "Solver survived with singular matrix and did not panic"
        );
    }
}
