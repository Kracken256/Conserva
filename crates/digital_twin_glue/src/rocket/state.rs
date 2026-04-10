use nalgebra::{Matrix3, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};
use uom::si::f64::{Angle, AngularVelocity, Force, Length, Mass, Time, Velocity};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissileState {
    pub time: Time,
    pub position: Vector3<Length>,
    pub body_velocity: Vector3<Velocity>,
    pub orientation: UnitQuaternion<f64>,
    pub angular_velocity: Vector3<AngularVelocity>,
    pub fin_angles: [Angle; 4],
    pub tvc_angles: [Angle; 2],
    pub current_mass: Mass,
    pub motor_thrust: Force,
    pub inertia_tensor: Matrix3<f64>,
}

pub trait RocketCtrl {
    fn update(&mut self, state: &MissileState, dt: f64) -> MissileState;
}
