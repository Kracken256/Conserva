use nalgebra::{Matrix3, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};
use uom::si::f32::{Angle, AngularVelocity, Length, Mass, Velocity};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissileState {
    pub position: Vector3<Length>,
    pub body_velocity: Vector3<Velocity>,
    pub orientation: UnitQuaternion<f32>,
    pub angular_velocity: Vector3<AngularVelocity>,
    pub fin_angles: [Angle; 4],
    pub tvc_angles: [Angle; 2],
    pub mass: Mass,
    pub inertia_tensor: Matrix3<f32>, // Keeping generic f32 for tensors as UOM complex inertia units are tough
}
