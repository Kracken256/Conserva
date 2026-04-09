use serde::{Deserialize, Serialize};
use uom::si::f32::{Force, Length, Time};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissileConfig {
    pub body_length: Length,
    pub diameter: Length,
    pub fin_offset_from_nose: Length,
    pub fin_chord_length: Length,
    pub motor_impulse_curve: Vec<(Time, Force)>,
}
