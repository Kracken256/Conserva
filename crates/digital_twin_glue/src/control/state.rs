use nalgebra::{Matrix3, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};
use uom::si::f64::{Angle, AngularVelocity, Length, Mass, Velocity};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissileState {
    pub position: Vector3<Length>,
    pub body_velocity: Vector3<Velocity>,
    pub orientation: UnitQuaternion<f64>,
    pub angular_velocity: Vector3<AngularVelocity>,
    pub fin_angles: [Angle; 4],
    pub tvc_angles: [Angle; 2],
    pub mass: Mass,
    pub inertia_tensor: Matrix3<f64>, // Keeping generic f64 for tensors as UOM complex inertia units are tough
}
