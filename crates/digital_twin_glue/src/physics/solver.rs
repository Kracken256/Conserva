use crate::control::state::MissileState;
use crate::geometry::mesh::Mesh;
use nalgebra::Vector3;

pub struct SolverOutput {
    pub force: Vector3<f32>,
    pub torque: Vector3<f32>,
}

pub trait AeroSolver {
    fn calculate_forces(&mut self, mesh: &Mesh, state: &MissileState) -> SolverOutput;
}
