use digital_twin_glue::prelude::*;
use nalgebra::Vector3;
use uom::si::length::meter;

#[derive(Debug, Clone, Default)]
pub struct TheMeshGenerator {}

impl MeshGenerator for TheMeshGenerator {
    fn generate(&self, _state: &MissileState, config: &MissileConfig, mesh: &mut Mesh) {
        let radius = config.geometry.diameter.get::<meter>() / 2.0;
        let body_length = config.geometry.body_length.get::<meter>();
        let fin_offset = config.geometry.fin_offset_from_nose.get::<meter>();
        let fin_chord = config.geometry.fin_chord_length.get::<meter>();
        let span = radius * 3.0; // Reasonable fin stick-out distance
        let fin_thickness = radius * 0.05_f64.max(0.001); // 5% of radius or minimum 1mm

        let nose_length = config.geometry.diameter.get::<meter>() * 2.0; // Typical rocket nose

        // Calculate dynamic Center of Gravity (CoG)
        let dry_m = _state.dry_mass.value;
        let prop_m = _state.propellant_mass.value;
        let cg_z = (0.0 * dry_m + (-body_length / 4.0) * prop_m) / (dry_m + prop_m);

        // Apply CoG shift to all root Z anchors
        let top_z = body_length / 2.0 - cg_z;
        let base_z = -body_length / 2.0 - cg_z;

        let sectors = 64; // High fidelity cylinder
        let nose_stacks = 32; // High fidelity ogive nose

        mesh.vertices.clear();
        mesh.indices.clear();

        let pi = std::f64::consts::PI;

        // --- 1. Nose Cone (Tangent Ogive) ---
        // R = (r^2 + L^2) / (2r)
        let ogive_radius = (radius * radius + nose_length * nose_length) / (2.0 * radius);

        let tip_idx = mesh.vertices.len() as u32;
        mesh.vertices.push(Vector3::new(0.0, 0.0, top_z));

        let mut prev_ring_idx = tip_idx;
        let mut prev_ring_size = 1;

        for i in 1..=nose_stacks {
            let t = i as f64 / nose_stacks as f64; // 0.0 to 1.0 (1.0 is base of nose)
            let x_ogive = nose_length * t; // distance from tip downwards
            // y = sqrt(R^2 - (L - x)^2) + r - R
            let y_ogive = {
                let inner = ogive_radius * ogive_radius - (nose_length - x_ogive).powi(2);
                if inner > 0.0 {
                    inner.sqrt() + radius - ogive_radius
                } else {
                    0.0 // Fail safe
                }
            };

            let current_z = top_z - x_ogive;
            let current_radius = y_ogive;

            let current_ring_idx = mesh.vertices.len() as u32;
            for j in 0..sectors {
                let angle = 2.0 * pi * (j as f64) / (sectors as f64);
                mesh.vertices.push(Vector3::new(
                    current_radius * angle.cos(),
                    current_radius * angle.sin(),
                    current_z,
                ));
            }

            // Stitch to previous ring
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

            prev_ring_idx = current_ring_idx;
            prev_ring_size = sectors;
        }

        let cyl_top_idx = prev_ring_idx; // The last ring of the nose is the top of the cylinder

        // --- 2. Main Cylinder ---
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

        // --- 3. Base Cap ---
        let base_center_idx = mesh.vertices.len() as u32;
        mesh.vertices.push(Vector3::new(0.0, 0.0, base_z));
        for j in 0..sectors {
            let j_next = (j + 1) % sectors;
            mesh.indices.push(base_center_idx);
            mesh.indices.push(cyl_base_idx + j_next);
            mesh.indices.push(cyl_base_idx + j);
        }

        // --- 4. 3D Delta Fins ---
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
        let state = get_initial_state();
        let config = get_default_config();
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
    fn mesh_total_length_equals_body_length() {
        let state = get_initial_state();
        let config = get_default_config();
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
        let expected_length = config.geometry.body_length.value;

        // Allow a tiny floating-point margin
        assert!(
            (calculated_length - expected_length).abs() < 1e-6,
            "Mesh length {} does not match expected body length {}",
            calculated_length,
            expected_length
        );
    }

    #[test]
    fn mesh_maximum_radius_includes_fins() {
        let state = get_initial_state();
        let config = get_default_config();
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
        let state = get_initial_state();
        let config = get_default_config();
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
