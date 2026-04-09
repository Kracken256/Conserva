use crate::control::state::MissileState;

pub trait Rocket {
    fn update(&mut self, state: &MissileState, dt: f64) -> MissileState;
}
