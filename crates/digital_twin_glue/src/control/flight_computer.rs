use crate::control::state::MissileState;

pub trait FlightComputer {
    fn update(&mut self, state: &MissileState, dt: f32) -> MissileState;
}
