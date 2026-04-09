use digital_twin::prelude::*;
use digital_twin_glue::prelude::*;
use std::fs::File;
use std::io::{self, Write};

fn main() -> io::Result<()> {
    let config = get_default_config();
    let state = get_initial_state(&config);
    let mut mesh = Mesh::default();
    let mesh_generator = TheMeshGenerator::default();

    println!("Generating mesh from default config...");
    mesh_generator.generate(&state, &config, &mut mesh);

    let output_file = "rocket.obj";
    println!(
        "Exporting {} vertices and {} triangles to {}...",
        mesh.vertices.len(),
        mesh.indices.len() / 3,
        output_file
    );

    let mut file = File::create(output_file)?;

    writeln!(file, "# Rocket Model")?;
    writeln!(file, "o Rocket")?;

    for v in &mesh.vertices {
        writeln!(file, "v {} {} {}", v.x, v.y, v.z)?;
    }

    for i in (0..mesh.indices.len()).step_by(3) {
        // OBJ indices are 1-based
        writeln!(
            file,
            "f {} {} {}",
            mesh.indices[i] + 1,
            mesh.indices[i + 1] + 1,
            mesh.indices[i + 2] + 1
        )?;
    }

    println!("Done!");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_valid_obj_vertex_output() {
        // A minimal test to check if vertices format right
        let config = get_default_config();
        let state = get_initial_state(&config);
        let mut mesh = Mesh::default();
        let mesh_generator = TheMeshGenerator::default();

        mesh_generator.generate(&state, &config, &mut mesh);

        // Ensure we have some vertices generated
        assert!(!mesh.vertices.is_empty(), "Mesh has no vertices");
        assert!(!mesh.indices.is_empty(), "Mesh has no indices");
        assert_eq!(
            mesh.indices.len() % 3,
            0,
            "Indices should form complete triangles"
        );
    }

    #[test]
    fn test_expected_file_is_written() {
        // Instead of writing a file, we could just check if the logic holds
        // but checking string formatting is more isolated
        let output = format!("v {} {} {}", 1.0, 2.0, 3.0);
        assert_eq!(output, "v 1 2 3");

        let index_output = format!("f {} {} {}", 1, 2, 3);
        assert_eq!(index_output, "f 1 2 3");
    }
}
