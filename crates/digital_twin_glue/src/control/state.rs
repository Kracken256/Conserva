use nalgebra::{Matrix3, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};
use uom::si::f64::{Angle, AngularVelocity, Force, Length, Mass, Velocity};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissileState {
    pub position: Vector3<Length>,
    pub body_velocity: Vector3<Velocity>,
    pub orientation: UnitQuaternion<f64>,
    pub angular_velocity: Vector3<AngularVelocity>,
    pub fin_angles: [Angle; 4],
    pub tvc_angles: [Angle; 2],
    pub dry_mass: Mass,
    pub propellant_mass: Mass,
    pub motor_thrust: Force,
    pub inertia_tensor: Matrix3<f64>, // Keeping generic f64 for tensors as UOM complex inertia units are tough
}

impl MissileState {
    pub fn total_mass(&self) -> Mass {
        self.dry_mass + self.propellant_mass
    }
}
