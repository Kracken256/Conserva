use digital_twin_glue::prelude::*;
use nalgebra::Vector3;

#[derive(Debug, Clone, Default)]
pub struct TheMeshGenerator {}

impl MeshGenerator for TheMeshGenerator {
    fn generate(&self, _state: &MissileState, _config: &MissileConfig, mesh: &mut Mesh) {
        let s = 1.5; // Half of 3 meters (for a 3x3x3m cube)

        mesh.vertices = vec![
            Vector3::new(-s, -s, -s), // 0
            Vector3::new(s, -s, -s),  // 1
            Vector3::new(s, s, -s),   // 2
            Vector3::new(-s, s, -s),  // 3
            Vector3::new(-s, -s, s),  // 4
            Vector3::new(s, -s, s),   // 5
            Vector3::new(s, s, s),    // 6
            Vector3::new(-s, s, s),   // 7
        ];

        // Define the 12 triangles (3 indices each) for the 6 faces.
        // Winding order is counter-clockwise looking from the outside.
        mesh.indices = vec![
            // Z- face
            0, 3, 2, 0, 2, 1, // Z+ face
            4, 5, 6, 4, 6, 7, // X- face
            0, 4, 7, 0, 7, 3, // X+ face
            1, 2, 6, 1, 6, 5, // Y- face
            0, 1, 5, 0, 5, 4, // Y+ face
            3, 7, 6, 3, 6, 2,
        ];
    }
}
