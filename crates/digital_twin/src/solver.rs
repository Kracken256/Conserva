use digital_twin_glue::prelude::*;

#[derive(Debug, Clone, Default)]
pub struct TheSolver {}

impl AeroSolver for TheSolver {
    fn calculate_forces(&mut self, mesh: &Mesh, state: &MissileState) -> SolverOutput {
        // TODO: Implement the solver logic here. For now, just return zero forces and torques.
        SolverOutput {
            force: [0.0, 0.0, 0.0].into(),
            torque: [0.0, 0.0, 0.0].into(),
        }
    }
}
