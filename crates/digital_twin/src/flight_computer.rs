use digital_twin_glue::prelude::*;

#[derive(Debug, Clone, Default)]
pub struct TheFlightComputer {}

impl FlightComputer for TheFlightComputer {
    fn update(&mut self, state: &MissileState, dt: f32) -> MissileState {
        // TODO: Write flight computer logic here. For now, just return the input state.
        state.clone()
    }
}
