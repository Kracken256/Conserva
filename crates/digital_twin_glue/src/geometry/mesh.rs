use crate::control::state::MissileState;
use crate::geometry::config::MissileConfig;
use nalgebra::Vector3;

#[derive(Clone, Default, Debug)]
pub struct MeshFace {
    pub normal: Vector3<f64>,
    pub centroid: Vector3<f64>,
    pub area: f64,
}

#[derive(Clone, Default)]
pub struct Mesh {
    pub vertices: Vec<Vector3<f64>>,
    pub indices: Vec<u32>,
    pub faces: Vec<MeshFace>,
}

impl Mesh {
    /// Precomputes surface normals, areas, and centroids for all triangles.
    /// This drastically reduces repetitive calculations during physics integration.
    pub fn compute_surface_properties(&mut self) {
        self.faces.clear();
        self.faces.reserve(self.indices.len() / 3);

        for chunk in self.indices.chunks_exact(3) {
            let v0 = self.vertices[chunk[0] as usize];
            let v1 = self.vertices[chunk[1] as usize];
            let v2 = self.vertices[chunk[2] as usize];

            let edge1 = v1 - v0;
            let edge2 = v2 - v0;
            let cross = edge1.cross(&edge2);
            let area = cross.magnitude() * 0.5;

            // Compute geometric properties or default to zero
            let (normal, centroid) = if area > 1e-8 {
                (cross.normalize(), (v0 + v1 + v2) / 3.0)
            } else {
                (Vector3::zeros(), Vector3::zeros())
            };

            self.faces.push(MeshFace {
                normal,
                centroid,
                area,
            });
        }
    }
}

pub trait MeshGenerator {
    fn generate(&self, state: &MissileState, config: &MissileConfig, mesh: &mut Mesh);
}
