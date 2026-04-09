use crate::control::state::MissileState;
use crate::geometry::config::MissileConfig;
use nalgebra::Vector3;

#[derive(Clone, Default, Debug)]
pub struct MeshFacesSoA {
    pub normal_x: Vec<f64>,
    pub normal_y: Vec<f64>,
    pub normal_z: Vec<f64>,
    pub centroid_x: Vec<f64>,
    pub centroid_y: Vec<f64>,
    pub centroid_z: Vec<f64>,
    pub area: Vec<f64>,
}

impl MeshFacesSoA {
    pub fn clear(&mut self) {
        self.normal_x.clear();
        self.normal_y.clear();
        self.normal_z.clear();
        self.centroid_x.clear();
        self.centroid_y.clear();
        self.centroid_z.clear();
        self.area.clear();
    }

    pub fn reserve(&mut self, capacity: usize) {
        self.normal_x.reserve(capacity);
        self.normal_y.reserve(capacity);
        self.normal_z.reserve(capacity);
        self.centroid_x.reserve(capacity);
        self.centroid_y.reserve(capacity);
        self.centroid_z.reserve(capacity);
        self.area.reserve(capacity);
    }
}

#[derive(Clone, Default)]
pub struct Mesh {
    pub vertices: Vec<Vector3<f64>>,
    pub indices: Vec<u32>,
    pub faces: MeshFacesSoA,
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

            self.faces.normal_x.push(normal.x);
            self.faces.normal_y.push(normal.y);
            self.faces.normal_z.push(normal.z);
            self.faces.centroid_x.push(centroid.x);
            self.faces.centroid_y.push(centroid.y);
            self.faces.centroid_z.push(centroid.z);
            self.faces.area.push(area);
        }
    }
}

pub trait MeshGenerator {
    fn generate(&self, state: &MissileState, config: &MissileConfig, mesh: &mut Mesh);
}
