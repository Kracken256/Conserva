use crate::control::state::MissileState;
use crate::geometry::config::MissileConfig;
use nalgebra::Vector3;

#[derive(Clone, Default)]
pub struct Mesh {
    pub vertices: Vec<Vector3<f64>>,
    pub indices: Vec<u32>,
}

pub trait MeshGenerator {
    fn generate(&self, state: &MissileState, config: &MissileConfig, mesh: &mut Mesh);
}
