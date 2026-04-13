use crate::prelude::*;
use nalgebra::Vector3;
use wide::{CmpGt, f64x8};

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
        let num_faces = self.indices.len() / 3;
        self.faces.reserve(num_faces);

        let mut chunks = self.indices.chunks_exact(24);
        for chunk in chunks.by_ref() {
            let mut v0_x = [0.0; 8];
            let mut v0_y = [0.0; 8];
            let mut v0_z = [0.0; 8];
            let mut v1_x = [0.0; 8];
            let mut v1_y = [0.0; 8];
            let mut v1_z = [0.0; 8];
            let mut v2_x = [0.0; 8];
            let mut v2_y = [0.0; 8];
            let mut v2_z = [0.0; 8];

            for i in 0..8 {
                let v0 = self.vertices[chunk[i * 3] as usize];
                let v1 = self.vertices[chunk[i * 3 + 1] as usize];
                let v2 = self.vertices[chunk[i * 3 + 2] as usize];

                v0_x[i] = v0.x;
                v0_y[i] = v0.y;
                v0_z[i] = v0.z;
                v1_x[i] = v1.x;
                v1_y[i] = v1.y;
                v1_z[i] = v1.z;
                v2_x[i] = v2.x;
                v2_y[i] = v2.y;
                v2_z[i] = v2.z;
            }

            let v0_x = f64x8::new(v0_x);
            let v0_y = f64x8::new(v0_y);
            let v0_z = f64x8::new(v0_z);
            let v1_x = f64x8::new(v1_x);
            let v1_y = f64x8::new(v1_y);
            let v1_z = f64x8::new(v1_z);
            let v2_x = f64x8::new(v2_x);
            let v2_y = f64x8::new(v2_y);
            let v2_z = f64x8::new(v2_z);

            let e1_x = v1_x - v0_x;
            let e1_y = v1_y - v0_y;
            let e1_z = v1_z - v0_z;

            let e2_x = v2_x - v0_x;
            let e2_y = v2_y - v0_y;
            let e2_z = v2_z - v0_z;

            let cx = e1_y * e2_z - e1_z * e2_y;
            let cy = e1_z * e2_x - e1_x * e2_z;
            let cz = e1_x * e2_y - e1_y * e2_x;

            let double_area_sq = cx * cx + cy * cy + cz * cz;
            let len = double_area_sq.sqrt();
            let area = len * f64x8::splat(0.5);

            let inv_len = f64x8::splat(1.0) / len;
            let valid = area.simd_gt(f64x8::splat(1e-8));

            let nx = cx * inv_len;
            let ny = cy * inv_len;
            let nz = cz * inv_len;

            let three = f64x8::splat(3.0);
            let cent_x = (v0_x + v1_x + v2_x) / three;
            let cent_y = (v0_y + v1_y + v2_y) / three;
            let cent_z = (v0_z + v1_z + v2_z) / three;

            let nx_arr = valid.blend(nx, f64x8::splat(0.0)).to_array();
            let ny_arr = valid.blend(ny, f64x8::splat(0.0)).to_array();
            let nz_arr = valid.blend(nz, f64x8::splat(0.0)).to_array();

            let ctx_arr = valid.blend(cent_x, f64x8::splat(0.0)).to_array();
            let cty_arr = valid.blend(cent_y, f64x8::splat(0.0)).to_array();
            let ctz_arr = valid.blend(cent_z, f64x8::splat(0.0)).to_array();

            let area_arr = valid.blend(area, f64x8::splat(0.0)).to_array();

            self.faces.normal_x.extend_from_slice(&nx_arr);
            self.faces.normal_y.extend_from_slice(&ny_arr);
            self.faces.normal_z.extend_from_slice(&nz_arr);
            self.faces.centroid_x.extend_from_slice(&ctx_arr);
            self.faces.centroid_y.extend_from_slice(&cty_arr);
            self.faces.centroid_z.extend_from_slice(&ctz_arr);
            self.faces.area.extend_from_slice(&area_arr);
        }

        for chunk in chunks.remainder().chunks_exact(3) {
            let v0 = self.vertices[chunk[0] as usize];
            let v1 = self.vertices[chunk[1] as usize];
            let v2 = self.vertices[chunk[2] as usize];

            let edge1 = v1 - v0;
            let edge2 = v2 - v0;
            let cross = edge1.cross(&edge2);
            let area = cross.magnitude() * 0.5;

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
