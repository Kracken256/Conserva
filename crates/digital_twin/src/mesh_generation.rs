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
        let fin = &config.geometry.fin_set;
        let offset = fin.offset_from_nose.get::<meter>();
        let root_c = fin.root_chord.get::<meter>();
        let tip_c = fin.tip_chord.get::<meter>();
        let span = fin.span.get::<meter>();
        let sweep = fin.sweep_length.get::<meter>();
        let thickness = fin.thickness.get::<meter>();
        let chamfer = fin.edge_chamfer.get::<meter>();
        let num_fins = fin.num_fins;

        let root_le_z = top_z - offset;
        let root_te_z = root_le_z - root_c;

        let tip_le_z = root_le_z - sweep;
        let tip_te_z = tip_le_z - tip_c;

        let le_depth = match fin.leading_edge_profile {
            FinEdgeProfile::Straight => 0.0,
            FinEdgeProfile::Curved { depth } => depth.get::<meter>(),
        };
        let te_depth = match fin.trailing_edge_profile {
            FinEdgeProfile::Straight => 0.0,
            FinEdgeProfile::Curved { depth } => depth.get::<meter>(),
        };

        // We slice the fin span into several stacks to give resolution for the curves.
        let fin_stacks = 10;

        for i in 0..num_fins {
            let angle = 2.0 * pi * (i as f64) / (num_fins as f64);
            let fw_x = angle.cos();
            let fw_y = angle.sin();
            let side_x = -angle.sin();
            let side_y = angle.cos();

            let half_t = thickness / 2.0;
            let fin_base_idx = mesh.vertices.len() as u32;

            for k in 0..=fin_stacks {
                let t = k as f64 / fin_stacks as f64;
                let r_k = radius + span * t;

                let uncurved_le = root_le_z * (1.0 - t) + tip_le_z * t;
                let uncurved_te = root_te_z * (1.0 - t) + tip_te_z * t;

                let bulge = 4.0 * t * (1.0 - t);
                let le_v = uncurved_le + le_depth * bulge;
                let te_v = uncurved_te - te_depth * bulge;

                let ch_l = if chamfer > 0.0 {
                    chamfer.min((le_v - te_v) / 2.0)
                } else {
                    0.0
                };

                // 6 points per slice ring to support chamfer
                // 0: LE tip (sharp)
                mesh.vertices
                    .push(Vector3::new(r_k * fw_x, r_k * fw_y, le_v));
                // 1: LE chamfer left
                mesh.vertices.push(Vector3::new(
                    r_k * fw_x - side_x * half_t,
                    r_k * fw_y - side_y * half_t,
                    le_v - ch_l,
                ));
                // 2: LE chamfer right
                mesh.vertices.push(Vector3::new(
                    r_k * fw_x + side_x * half_t,
                    r_k * fw_y + side_y * half_t,
                    le_v - ch_l,
                ));

                // 3: TE chamfer left
                mesh.vertices.push(Vector3::new(
                    r_k * fw_x - side_x * half_t,
                    r_k * fw_y - side_y * half_t,
                    te_v + ch_l,
                ));
                // 4: TE chamfer right
                mesh.vertices.push(Vector3::new(
                    r_k * fw_x + side_x * half_t,
                    r_k * fw_y + side_y * half_t,
                    te_v + ch_l,
                ));
                // 5: TE tip (sharp)
                mesh.vertices
                    .push(Vector3::new(r_k * fw_x, r_k * fw_y, te_v));

                if k > 0 {
                    let prev_idx = fin_base_idx + (k - 1) * 6;
                    let curr_idx = fin_base_idx + k * 6;

                    // LE left chamfer panel
                    mesh.indices.push(prev_idx + 0);
                    mesh.indices.push(prev_idx + 1);
                    mesh.indices.push(curr_idx + 1);
                    mesh.indices.push(prev_idx + 0);
                    mesh.indices.push(curr_idx + 1);
                    mesh.indices.push(curr_idx + 0);

                    // LE right chamfer panel
                    mesh.indices.push(prev_idx + 2);
                    mesh.indices.push(prev_idx + 0);
                    mesh.indices.push(curr_idx + 0);
                    mesh.indices.push(prev_idx + 2);
                    mesh.indices.push(curr_idx + 0);
                    mesh.indices.push(curr_idx + 2);

                    // Left mid panel
                    mesh.indices.push(prev_idx + 1);
                    mesh.indices.push(prev_idx + 3);
                    mesh.indices.push(curr_idx + 3);
                    mesh.indices.push(prev_idx + 1);
                    mesh.indices.push(curr_idx + 3);
                    mesh.indices.push(curr_idx + 1);

                    // Right mid panel
                    mesh.indices.push(prev_idx + 4);
                    mesh.indices.push(prev_idx + 2);
                    mesh.indices.push(curr_idx + 2);
                    mesh.indices.push(prev_idx + 4);
                    mesh.indices.push(curr_idx + 2);
                    mesh.indices.push(curr_idx + 4);

                    // TE left chamfer panel
                    mesh.indices.push(prev_idx + 3);
                    mesh.indices.push(prev_idx + 5);
                    mesh.indices.push(curr_idx + 5);
                    mesh.indices.push(prev_idx + 3);
                    mesh.indices.push(curr_idx + 5);
                    mesh.indices.push(curr_idx + 3);

                    // TE right chamfer panel
                    mesh.indices.push(prev_idx + 5);
                    mesh.indices.push(prev_idx + 4);
                    mesh.indices.push(curr_idx + 4);
                    mesh.indices.push(prev_idx + 5);
                    mesh.indices.push(curr_idx + 4);
                    mesh.indices.push(curr_idx + 5);
                }
            }

            // Cap the Top (Tip of the fin)
            let tip_idx = fin_base_idx + fin_stacks * 6;

            // Top LE wedge cap
            mesh.indices.push(tip_idx + 0);
            mesh.indices.push(tip_idx + 1);
            mesh.indices.push(tip_idx + 2);
            // Top mid quad cap
            mesh.indices.push(tip_idx + 1);
            mesh.indices.push(tip_idx + 3);
            mesh.indices.push(tip_idx + 4);
            mesh.indices.push(tip_idx + 1);
            mesh.indices.push(tip_idx + 4);
            mesh.indices.push(tip_idx + 2);
            // Top TE wedge cap
            mesh.indices.push(tip_idx + 3);
            mesh.indices.push(tip_idx + 5);
            mesh.indices.push(tip_idx + 4);
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
        let expected_span = config.geometry.fin_set.span.value;
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
            (config.geometry.diameter.value / 2.0) + config.geometry.fin_set.span.value;

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

    #[test]
    fn mesh_indices_form_complete_triangles() {
        let config = get_default_config();
        let state = get_initial_state(&config);
        let mut mesh = Mesh::default();
        let generator = TheMeshGenerator::default();

        generator.generate(&state, &config, &mut mesh);

        assert_eq!(
            mesh.indices.len() % 3,
            0,
            "Indices count {} is not a multiple of 3 (not a valid triangle set)",
            mesh.indices.len()
        );
    }

    #[test]
    fn mesh_contains_no_degenerate_or_infinite_vertices() {
        let config = get_default_config();
        let state = get_initial_state(&config);
        let mut mesh = Mesh::default();
        let generator = TheMeshGenerator::default();

        generator.generate(&state, &config, &mut mesh);

        for (i, v) in mesh.vertices.iter().enumerate() {
            assert!(
                v.x.is_finite() && v.y.is_finite() && v.z.is_finite(),
                "Vertex {} is infinite or NaN: {:?}",
                i,
                v
            );
        }
    }

    #[test]
    fn center_of_gravity_shift_moves_entire_mesh_purely_in_z() {
        use uom::si::f64::{Length, Time};
        use uom::si::length::meter;
        use uom::si::time::second;

        let mut config_cg_zero = get_default_config();

        // Modify cg curve to strictly returning 0.0 at all times
        config_cg_zero.geometry.cg_curve = vec![(
            Time::new::<second>(0.0),
            Vector3::new(
                Length::new::<meter>(0.0),
                Length::new::<meter>(0.0),
                Length::new::<meter>(0.0),
            ),
        )];

        let state = get_initial_state(&config_cg_zero);
        let mut mesh_cg_zero = Mesh::default();
        let generator = TheMeshGenerator::default();
        generator.generate(&state, &config_cg_zero, &mut mesh_cg_zero);

        let mut config_cg_shifted = config_cg_zero.clone();
        config_cg_shifted.geometry.cg_curve = vec![(
            Time::new::<second>(0.0),
            Vector3::new(
                Length::new::<meter>(0.0),
                Length::new::<meter>(0.0),
                Length::new::<meter>(1.0),
            ),
        )];

        let mut mesh_cg_shifted = Mesh::default();
        generator.generate(&state, &config_cg_shifted, &mut mesh_cg_shifted);

        assert_eq!(mesh_cg_zero.vertices.len(), mesh_cg_shifted.vertices.len());

        for (v0, v1) in mesh_cg_zero
            .vertices
            .iter()
            .zip(mesh_cg_shifted.vertices.iter())
        {
            assert_eq!(v0.x, v1.x);
            assert_eq!(v0.y, v1.y);
            // v1 should be strictly translated backwards by exactly 1.0m
            assert!((v1.z - (v0.z - 1.0)).abs() < 1e-6);
        }
    }

    #[test]
    fn mesh_generates_correct_number_of_fins() {
        let mut config = get_default_config();
        // Base vertex count without fins is the cylinder/nose components
        config.geometry.fin_set.num_fins = 0;
        let state = get_initial_state(&config);
        let mut mesh_zero = Mesh::default();
        let generator = TheMeshGenerator::default();
        generator.generate(&state, &config, &mut mesh_zero);

        let count_zero = mesh_zero.vertices.len();

        config.geometry.fin_set.num_fins = 4;
        let mut mesh_four = Mesh::default();
        generator.generate(&state, &config, &mut mesh_four);
        let count_four = mesh_four.vertices.len();

        assert!(count_four > count_zero, "Expected more vertices for 4 fins");

        let fins_vertices = count_four - count_zero;

        config.geometry.fin_set.num_fins = 8;
        let mut mesh_eight = Mesh::default();
        generator.generate(&state, &config, &mut mesh_eight);
        let count_eight = mesh_eight.vertices.len();

        let fins_vertices_8 = count_eight - count_zero;
        assert_eq!(
            fins_vertices * 2,
            fins_vertices_8,
            "8 fins should generate twice as many fin vertices as 4 fins"
        );
    }

    #[test]
    fn blunted_nosecone_generates_larger_tip_radius() {
        let mut config_sharp = get_default_config();
        use uom::si::f64::Length;
        use uom::si::length::meter;
        config_sharp.geometry.nosecone_shape = NoseconeShape::Conical {
            length: Length::new::<meter>(1.0),
            blunting_radius: None,
        };

        let mut config_blunt = config_sharp.clone();
        config_blunt.geometry.nosecone_shape = NoseconeShape::Conical {
            length: Length::new::<meter>(1.0),
            blunting_radius: Some(Length::new::<meter>(0.1)),
        };

        let radius_sharp =
            TheMeshGenerator::apply_blunting(&config_sharp.geometry.nosecone_shape, 0.001, 0.05);
        let radius_blunt =
            TheMeshGenerator::apply_blunting(&config_blunt.geometry.nosecone_shape, 0.001, 0.05);

        // Blunted profile effectively rounds off the sharp conical tip
        assert!(radius_blunt > radius_sharp);
    }

    #[test]
    fn different_nosecone_shapes_yield_unique_profiles() {
        use uom::si::f64::Length;
        use uom::si::length::meter;
        let base_radius = 0.1;
        let length = 1.0;
        let t = 0.5; // Midway up the nose
        let x = 0.5; // 0.5 meters down

        let conical = NoseconeShape::Conical {
            length: Length::new::<meter>(length),
            blunting_radius: None,
        };
        let r_conical =
            TheMeshGenerator::calculate_profile_radius(&conical, base_radius, length, x, t);

        let elliptic = NoseconeShape::Elliptical {
            length: Length::new::<meter>(length),
        };
        let r_elliptic =
            TheMeshGenerator::calculate_profile_radius(&elliptic, base_radius, length, x, t);

        let power = NoseconeShape::PowerSeries {
            length: Length::new::<meter>(length),
            blunting_radius: None,
            n: 0.75,
        };
        let r_power = TheMeshGenerator::calculate_profile_radius(&power, base_radius, length, x, t);

        assert!(
            (r_conical - 0.05).abs() < 1e-6,
            "Conical at t=0.5 should be half base radius"
        );

        // Elliptical should be wider/fatter in the middle than a straight conical line
        assert!(r_elliptic > r_conical);

        // Power series with n=0.75 is 0.5^0.75 * 0.1 = 0.0594, wider than cone but probably narrower than ellipse
        assert!(r_power > r_conical);
        assert!(
            (r_elliptic - r_power).abs() > 1e-3,
            "Shapes should produce unique radiuses"
        );
    }
}
