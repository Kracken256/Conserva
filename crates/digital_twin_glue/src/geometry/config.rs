use serde::{Deserialize, Serialize};
use uom::si::f64::{Force, Length, Time};

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
pub struct MissileConfig {
    pub geometry: MissileGeometryConfig,
    pub controller: MissileControllerConfig,
    pub engine: MissileEngineConfig,
}
