use digital_twin::prelude::*;
use digital_twin_glue::prelude::*;
use std::fs::File;
use std::io::{self, Write};

fn main() -> io::Result<()> {
    let state = get_initial_state();
    let config = get_default_config();
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
