use digital_twin_glue::prelude::*;
use nalgebra::Vector3;
use uom::si::length::meter;

#[derive(Debug, Clone, Default)]
pub struct TheMeshGenerator {}

impl MeshGenerator for TheMeshGenerator {
    fn generate(&self, _state: &MissileState, config: &MissileConfig, mesh: &mut Mesh) {
        let radius = config.diameter.get::<meter>() / 2.0;
        let body_length = config.body_length.get::<meter>();
        let fin_offset = config.fin_offset_from_nose.get::<meter>();
        let fin_chord = config.fin_chord_length.get::<meter>();
        let span = radius * 3.0; // Reasonable fin stick-out distance
        let fin_thickness = radius * 0.05_f64.max(0.001); // 5% of radius or minimum 1mm

        let nose_length = config.diameter.get::<meter>() * 2.0; // Typical rocket nose

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
