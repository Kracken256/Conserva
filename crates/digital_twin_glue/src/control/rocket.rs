use crate::control::state::MissileState;

pub trait Rocket {
    fn update(&mut self, state: &MissileState, dt: f32) -> MissileState;
}
