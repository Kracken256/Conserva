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
            &Vector3<f64>,
            &Vector3<f64>,
            &Quaternion<f64>,
            &Vector3<f64>,
        ) -> (Vector3<f64>, Vector3<f64>),
    {
        let q_u = UnitQuaternion::new_normalize(q);

        // 1. SAMPLE PHYSICS: Get forces for this specific intermediate state
        let (force_body, torque_body) = get_forces(&v, &w, &q, &p);

        // 2. KINEMATICS
        let dp = q_u * v;

        // 3. LINEAR DYNAMICS (Body Frame)
        let gravity_world = Vector3::new(0.0, 0.0, -9.81);
        let gravity_body = q_u.inverse() * gravity_world;
        let dv = (force_body / mass) + gravity_body - w.cross(&v);

        // 4. ROTATIONAL DYNAMICS (Body Frame)
        let dw = inertia_inv * (torque_body - w.cross(&(inertia * w)));

        // 5. ORIENTATION DYNAMICS
        let dq = 0.5 * Quaternion::new(0.0, w.x, w.y, w.z) * q;

        (dp, dv, dw, dq)
    }

    pub fn step<F>(&mut self, state: &MissileState, mut get_forces: F, dt: f64) -> MissileState
    where
        F: FnMut(
            &Vector3<f64>,
            &Vector3<f64>,
            &Quaternion<f64>,
            &Vector3<f64>,
        ) -> (Vector3<f64>, Vector3<f64>),
    {
        let mass = state.total_mass().value;
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
