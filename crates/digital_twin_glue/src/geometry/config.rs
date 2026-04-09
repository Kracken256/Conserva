use nalgebra::Matrix3;
use serde::{Deserialize, Serialize};
use uom::si::f64::{Force, Length, Mass, Time};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissileGeometryConfig {
    pub body_length: Length,
    pub diameter: Length,
    pub fin_offset_from_nose: Length,
    pub fin_chord_length: Length,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissileControllerConfig {
    pub pitch_pid_kp: f64,
    pub pitch_pid_ki: f64,
    pub pitch_pid_kd: f64,
    pub yaw_pid_kp: f64,
    pub yaw_pid_ki: f64,
    pub yaw_pid_kd: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissileEngineConfig {
    pub motor_impulse_curve: Vec<(Time, Force)>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissileMassConfig {
    pub dry_mass: Mass,
    pub wet_mass: Mass,
    pub mass_curve: Vec<(Time, Mass)>,
    pub inertia_tensor: Matrix3<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissileConfig {
    pub geometry: MissileGeometryConfig,
    pub controller: MissileControllerConfig,
    pub engine: MissileEngineConfig,
    pub mass: MissileMassConfig,
}
