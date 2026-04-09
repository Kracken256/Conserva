use digital_twin_glue::prelude::*;
use nalgebra::Vector3;

#[derive(Debug, Clone, Default)]
pub struct TheMeshGenerator {}

impl MeshGenerator for TheMeshGenerator {
    fn generate(&self, _state: &MissileState, _config: &MissileConfig, mesh: &mut Mesh) {
        let radius = 1.5;
        let sectors = 1024;
        let stacks = 1024;

        mesh.vertices.clear();
        mesh.indices.clear();

        let pi = std::f64::consts::PI;

        // Generate vertices
        for i in 0..=stacks {
            let stack_angle = pi / 2.0 - pi * (i as f64) / (stacks as f64);
            let xy = radius * stack_angle.cos();
            let z = radius * stack_angle.sin();

            for j in 0..=sectors {
                let sector_angle = 2.0 * pi * (j as f64) / (sectors as f64);
                let x = xy * sector_angle.cos();
                let y = xy * sector_angle.sin();
                mesh.vertices.push(Vector3::new(x, y, z));
            }
        }

        // Generate indices
        for i in 0..stacks {
            let mut k1 = i * (sectors + 1);
            let mut k2 = k1 + sectors + 1;

            for _j in 0..sectors {
                if i != 0 {
                    mesh.indices.push(k1);
                    mesh.indices.push(k2);
                    mesh.indices.push(k1 + 1);
                }

                if i != (stacks - 1) {
                    mesh.indices.push(k1 + 1);
                    mesh.indices.push(k2);
                    mesh.indices.push(k2 + 1);
                }

                k1 += 1;
                k2 += 1;
            }
        }
    }
}
