use digital_twin_glue::prelude::*;
use nalgebra::Vector3;
use uom::si::length::meter;

#[derive(Debug, Clone, Default)]
pub struct TheMeshGenerator {}

impl MeshGenerator for TheMeshGenerator {
    fn generate(&self, state: &MissileState, config: &MissileConfig, mesh: &mut Mesh) {
        let radius = config.geometry.diameter.get::<meter>() / 2.0;
        let body_length = config.geometry.cylindrical_body_length.get::<meter>();
        let nose_length = config.geometry.nosecone_shape.length().get::<meter>();
        let total_length = body_length + nose_length;

        // Fetch dynamic Center of Gravity (CoG) from curve
        let cg = config.geometry.current_cg(state.time);

        // Apply CoG shift to all root Z anchors
        let top_z = total_length / 2.0 - cg.z.get::<meter>();
        let base_z = -total_length / 2.0 - cg.z.get::<meter>();

        let sectors = 64; // High fidelity cylinder
        let nose_stacks = 32; // High fidelity nose

        mesh.vertices.clear();
        mesh.indices.clear();

        let cyl_top_idx =
            self.generate_nose_cone(config, radius, top_z, sectors, nose_stacks, mesh);
        let cyl_base_idx = self.generate_main_cylinder(radius, base_z, sectors, cyl_top_idx, mesh);
        self.generate_base_cap(base_z, sectors, cyl_base_idx, mesh);
        self.generate_fins(config, radius, top_z, mesh);
    }
}

impl TheMeshGenerator {
    fn generate_nose_cone(
        &self,
        config: &MissileConfig,
        radius: f64,
        top_z: f64,
        sectors: u32,
        nose_stacks: u32,
        mesh: &mut Mesh,
    ) -> u32 {
        let nose_length = config.geometry.nosecone_shape.length().get::<meter>();
        let nose_shape = &config.geometry.nosecone_shape;

        let tip_idx = mesh.vertices.len() as u32;
        mesh.vertices.push(Vector3::new(0.0, 0.0, top_z));

        let mut prev_ring_idx = tip_idx;
        let mut prev_ring_size = 1;

        for i in 1..=nose_stacks {
            let t = i as f64 / nose_stacks as f64; // 0.0 to 1.0 (1.0 is base of nose)
            let x = nose_length * t; // distance from tip downwards

            let current_radius =
                Self::calculate_profile_radius(nose_shape, radius, nose_length, x, t);
            let current_radius = Self::apply_blunting(nose_shape, current_radius, x);

            let current_z = top_z - x;
            let current_ring_idx = mesh.vertices.len() as u32;

            let pi = std::f64::consts::PI;
            for j in 0..sectors {
                let angle = 2.0 * pi * (j as f64) / (sectors as f64);
                mesh.vertices.push(Vector3::new(
                    current_radius * angle.cos(),
                    current_radius * angle.sin(),
                    current_z,
                ));
            }

            Self::stitch_nose_ring(
                mesh,
                sectors,
                tip_idx,
                prev_ring_idx,
                current_ring_idx,
                prev_ring_size,
            );

            prev_ring_idx = current_ring_idx;
            prev_ring_size = sectors;
        }

        prev_ring_idx // The last ring of the nose is the top of the cylinder
    }

    fn calculate_profile_radius(
        nose_shape: &NoseconeShape,
        base_radius: f64,
        nose_length: f64,
        x: f64,
        t: f64,
    ) -> f64 {
        let pi = std::f64::consts::PI;
        match nose_shape {
            NoseconeShape::Conical { .. } => base_radius * t,
            NoseconeShape::Elliptical { .. } => {
                let d = (nose_length - x) / nose_length;
                base_radius * (1.0 - d * d).max(0.0).sqrt()
            }
            NoseconeShape::Parabolic { k_factor, .. } => {
                base_radius * (2.0 * t - k_factor * t * t) / (2.0 - k_factor)
            }
            NoseconeShape::PowerSeries { n, .. } => base_radius * t.powf(*n),
            NoseconeShape::Haack { c_factor, .. } => {
                let theta = (1.0 - 2.0 * t).acos();
                let y = (theta - (2.0 * theta).sin() / 2.0 + c_factor * theta.sin().powi(3)) / pi;
                base_radius * y.max(0.0).sqrt()
            }
            NoseconeShape::Ogive { secant_radius, .. } => {
                let rho = if let Some(sr) = secant_radius {
                    sr.get::<meter>()
                } else {
                    (base_radius * base_radius + nose_length * nose_length) / (2.0 * base_radius)
                };

                let inner = rho * rho - (nose_length - x).powi(2);
                if inner > 0.0 {
                    inner.sqrt() + base_radius - rho
                } else {
                    0.0
                }
            }
        }
    }

    fn apply_blunting(nose_shape: &NoseconeShape, mut current_radius: f64, x: f64) -> f64 {
        let blunting_r = match nose_shape {
            NoseconeShape::Conical {
                blunting_radius, ..
            }
            | NoseconeShape::Ogive {
                blunting_radius, ..
            }
            | NoseconeShape::Parabolic {
                blunting_radius, ..
            }
            | NoseconeShape::PowerSeries {
                blunting_radius, ..
            }
            | NoseconeShape::Haack {
                blunting_radius, ..
            } => blunting_radius.map(|l| l.get::<meter>()),
            NoseconeShape::Elliptical { .. } => None,
        };

        if let Some(rn) = blunting_r {
            if x < rn {
                current_radius = current_radius.max((rn * rn - (rn - x).powi(2)).max(0.0).sqrt());
            }
        }
        current_radius
    }

    fn stitch_nose_ring(
        mesh: &mut Mesh,
        sectors: u32,
        tip_idx: u32,
        prev_ring_idx: u32,
        current_ring_idx: u32,
        prev_ring_size: u32,
    ) {
        if prev_ring_size == 1 {
            // Stitch tip to first ring
            for j in 0..sectors {
                let j_next = (j + 1) % sectors;
                mesh.indices.push(tip_idx);
                mesh.indices.push(current_ring_idx + j);
                mesh.indices.push(current_ring_idx + j_next);
            }
        } else {
            // Stitch ring to ring
            for j in 0..sectors {
                let j_next = (j + 1) % sectors;
                let top1 = prev_ring_idx + j;
                let top2 = prev_ring_idx + j_next;
                let bot1 = current_ring_idx + j;
                let bot2 = current_ring_idx + j_next;

                mesh.indices.push(top1);
                mesh.indices.push(bot1);
                mesh.indices.push(top2);

                mesh.indices.push(bot1);
                mesh.indices.push(bot2);
                mesh.indices.push(top2);
            }
        }
    }

    fn generate_main_cylinder(
        &self,
        radius: f64,
        base_z: f64,
        sectors: u32,
        cyl_top_idx: u32,
        mesh: &mut Mesh,
    ) -> u32 {
        let pi = std::f64::consts::PI;
        let cyl_base_idx = mesh.vertices.len() as u32;

        for j in 0..sectors {
            let angle = 2.0 * pi * (j as f64) / (sectors as f64);
            mesh.vertices.push(Vector3::new(
                radius * angle.cos(),
                radius * angle.sin(),
                base_z,
            ));
        }

        // Cylinder body indices
        for j in 0..sectors {
            let j_next = (j + 1) % sectors;
            let top1 = cyl_top_idx + j;
            let top2 = cyl_top_idx + j_next;
            let bot1 = cyl_base_idx + j;
            let bot2 = cyl_base_idx + j_next;

            mesh.indices.push(top1);
            mesh.indices.push(bot1);
            mesh.indices.push(top2);

            mesh.indices.push(bot1);
            mesh.indices.push(bot2);
            mesh.indices.push(top2);
        }

        cyl_base_idx
    }

    fn generate_base_cap(&self, base_z: f64, sectors: u32, cyl_base_idx: u32, mesh: &mut Mesh) {
        let base_center_idx = mesh.vertices.len() as u32;
        mesh.vertices.push(Vector3::new(0.0, 0.0, base_z));
        for j in 0..sectors {
            let j_next = (j + 1) % sectors;
            mesh.indices.push(base_center_idx);
            mesh.indices.push(cyl_base_idx + j_next);
            mesh.indices.push(cyl_base_idx + j);
        }
    }

    fn generate_fins(&self, config: &MissileConfig, radius: f64, top_z: f64, mesh: &mut Mesh) {
        let pi = std::f64::consts::PI;
        let fin_offset = config.geometry.fin_offset_from_nose.get::<meter>();
        let fin_chord = config.geometry.fin_chord_length.get::<meter>();
        let span = radius * 3.0; // Reasonable fin stick-out distance
        let fin_thickness = radius * 0.05_f64.max(0.001); // 5% of radius or minimum 1mm

        let num_fins = 4;
        let fin_top_z = top_z - fin_offset;
        let fin_bot_z = fin_top_z - fin_chord;

        for i in 0..num_fins {
            let angle = 2.0 * pi * (i as f64) / (num_fins as f64);

            // Perpendicular vector for thickness
            let fw_x = angle.cos();
            let fw_y = angle.sin();
            let side_x = -angle.sin();
            let side_y = angle.cos();

            let half_t = fin_thickness / 2.0;

            // Geometry of one fin: root top, root bot, tip tip
            // Left side (CCW with side vector subtracted)
            let fin_base_idx = mesh.vertices.len() as u32;

            // 0: Root Top Left
            mesh.vertices.push(Vector3::new(
                radius * fw_x - side_x * half_t,
                radius * fw_y - side_y * half_t,
                fin_top_z,
            ));
            // 1: Root Bot Left
            mesh.vertices.push(Vector3::new(
                radius * fw_x - side_x * half_t,
                radius * fw_y - side_y * half_t,
                fin_bot_z,
            ));
            // 2: Tip Left
            mesh.vertices.push(Vector3::new(
                (radius + span) * fw_x - side_x * half_t,
                (radius + span) * fw_y - side_y * half_t,
                fin_bot_z,
            ));

            // Right side (CCW with side vector added)
            // 3: Root Top Right
            mesh.vertices.push(Vector3::new(
                radius * fw_x + side_x * half_t,
                radius * fw_y + side_y * half_t,
                fin_top_z,
            ));
            // 4: Root Bot Right
            mesh.vertices.push(Vector3::new(
                radius * fw_x + side_x * half_t,
                radius * fw_y + side_y * half_t,
                fin_bot_z,
            ));
            // 5: Tip Right
            mesh.vertices.push(Vector3::new(
                (radius + span) * fw_x + side_x * half_t,
                (radius + span) * fw_y + side_y * half_t,
                fin_bot_z,
            ));

            // Left face (0, 1, 2) -> CCW
            mesh.indices.push(fin_base_idx + 0);
            mesh.indices.push(fin_base_idx + 1);
            mesh.indices.push(fin_base_idx + 2);

            // Right face (3, 5, 4) -> reversed to face outward
            mesh.indices.push(fin_base_idx + 3);
            mesh.indices.push(fin_base_idx + 5);
            mesh.indices.push(fin_base_idx + 4);

            // Top edge (0, 2, 3) & (2, 5, 3)
            mesh.indices.push(fin_base_idx + 0);
            mesh.indices.push(fin_base_idx + 2);
            mesh.indices.push(fin_base_idx + 3);
            mesh.indices.push(fin_base_idx + 2);
            mesh.indices.push(fin_base_idx + 5);
            mesh.indices.push(fin_base_idx + 3);

            // Back edge (1, 4, 2) & (4, 5, 2)
            mesh.indices.push(fin_base_idx + 1);
            mesh.indices.push(fin_base_idx + 4);
            mesh.indices.push(fin_base_idx + 2);
            mesh.indices.push(fin_base_idx + 4);
            mesh.indices.push(fin_base_idx + 5);
            mesh.indices.push(fin_base_idx + 2);

            // Root edge covered by main body
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::defaults::{get_default_config, get_initial_state};

    /// Ensure that the generated mesh treats +Z as the rocket's longitudinal
    /// axis, with the nose tip located at the maximum Z value and centered
    /// around X = Y = 0. This couples the geometric model to the body-frame
    /// convention used by the physics and controller.
    #[test]
    fn mesh_nose_is_aligned_with_body_z_axis() {
        let config = get_default_config();
        let state = get_initial_state(&config);
        let mut mesh = Mesh::default();
        let generator = TheMeshGenerator::default();

        generator.generate(&state, &config, &mut mesh);

        assert!(!mesh.vertices.is_empty(), "mesh should contain vertices");

        // Find the maximum Z value in the mesh.
        let max_z = mesh
            .vertices
            .iter()
            .map(|v| v.z)
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(max_z.is_finite());

        // Collect all vertices that are effectively at the nose tip (max Z).
        let tip_verts: Vec<_> = mesh
            .vertices
            .iter()
            .filter(|v| (max_z - v.z).abs() < 1e-6)
            .collect();

        assert!(
            !tip_verts.is_empty(),
            "expected at least one vertex at the nose tip (max Z)"
        );

        let avg_x = tip_verts.iter().map(|v| v.x).sum::<f64>() / tip_verts.len() as f64;
        let avg_y = tip_verts.iter().map(|v| v.y).sum::<f64>() / tip_verts.len() as f64;

        // Nose tip should lie very close to the +Z axis.
        assert!(avg_x.abs() < 1e-3, "nose tip x not near zero: {}", avg_x);
        assert!(avg_y.abs() < 1e-3, "nose tip y not near zero: {}", avg_y);
    }

    #[test]
    fn mesh_total_length_equals_body_plus_nose_length() {
        let config = get_default_config();
        let state = get_initial_state(&config);
        let mut mesh = Mesh::default();
        let generator = TheMeshGenerator::default();

        generator.generate(&state, &config, &mut mesh);

        let max_z = mesh
            .vertices
            .iter()
            .map(|v| v.z)
            .fold(f64::NEG_INFINITY, f64::max);
        let min_z = mesh
            .vertices
            .iter()
            .map(|v| v.z)
            .fold(f64::INFINITY, f64::min);
        let calculated_length = max_z - min_z;
        let expected_length = config.geometry.cylindrical_body_length.value
            + config.geometry.nosecone_shape.length().value;

        // Allow a tiny floating-point margin
        assert!(
            (calculated_length - expected_length).abs() < 1e-6,
            "Mesh length {} does not match expected total length {}",
            calculated_length,
            expected_length
        );
    }

    #[test]
    fn mesh_maximum_radius_includes_fins() {
        let config = get_default_config();
        let state = get_initial_state(&config);
        let mut mesh = Mesh::default();
        let generator = TheMeshGenerator::default();

        generator.generate(&state, &config, &mut mesh);

        // Find max distance from Z-axis
        let max_radius = mesh
            .vertices
            .iter()
            .map(|v| (v.x * v.x + v.y * v.y).sqrt())
            .fold(f64::NEG_INFINITY, f64::max);

        let expected_body_radius = config.geometry.diameter.value / 2.0;
        let expected_span = expected_body_radius * 3.0;
        let expected_max_radius = expected_body_radius + expected_span;

        assert!(
            (max_radius - expected_max_radius).abs() < 1e-2,
            "Max radius {} does not match expected body + fin span {}",
            max_radius,
            expected_max_radius
        );
    }

    #[test]
    fn mesh_generates_correct_volume_bounding_box() {
        let config = get_default_config();
        let state = get_initial_state(&config);
        let mut mesh = Mesh::default();
        let generator = TheMeshGenerator::default();

        generator.generate(&state, &config, &mut mesh);

        let max_x = mesh
            .vertices
            .iter()
            .map(|v| v.x)
            .fold(f64::NEG_INFINITY, f64::max);
        let min_x = mesh
            .vertices
            .iter()
            .map(|v| v.x)
            .fold(f64::INFINITY, f64::min);
        let max_y = mesh
            .vertices
            .iter()
            .map(|v| v.y)
            .fold(f64::NEG_INFINITY, f64::max);
        let min_y = mesh
            .vertices
            .iter()
            .map(|v| v.y)
            .fold(f64::INFINITY, f64::min);

        // Fins stick out uniformly in 4 directions, so max/min X and Y should be roughly equal to max radius
        let expected_bound =
            (config.geometry.diameter.value / 2.0) + (config.geometry.diameter.value / 2.0) * 3.0;

        assert!(
            (max_x - expected_bound).abs() < 1e-2,
            "X bound max mismatch"
        );
        assert!(
            (min_x - (-expected_bound)).abs() < 1e-2,
            "X bound min mismatch"
        );
        assert!(
            (max_y - expected_bound).abs() < 1e-2,
            "Y bound max mismatch"
        );
        assert!(
            (min_y - (-expected_bound)).abs() < 1e-2,
            "Y bound min mismatch"
        );
    }
}
