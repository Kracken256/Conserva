use digital_twin_glue::prelude::*;

#[derive(Debug, Clone, Default)]
pub struct TheMeshGenerator {}

impl MeshGenerator for TheMeshGenerator {
    fn generate(&self, state: &MissileState, config: &MissileConfig, mesh: &mut Mesh) {
        //TODO: generate the mesh based on the missile state and configuration. For now, this is just a placeholder.
    }
}
