use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissileConfig {
    pub body_length: f32,
    pub diameter: f32,
    pub fin_offset_from_nose: f32,
    pub fin_chord_length: f32,
    pub motor_impulse_curve: Vec<(f32, f32)>,
}
