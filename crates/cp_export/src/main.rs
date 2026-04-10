use digital_twin::prelude::*;
use digital_twin_glue::prelude::*;
use std::fs::File;
use std::io::Write;
use uom::si::f64::{Time, Velocity};
use uom::si::length::meter;
use uom::si::time::second;
use uom::si::velocity::meter_per_second;

fn main() -> std::io::Result<()> {
    let config = get_default_config();
    let mesh_generator = TheMeshGenerator::default();
    let body_length = config.geometry.body_length.get::<meter>();

    println!("Exporting center of pressure curve...");
    let mut cp_file = File::create("cp_curve.csv")?;
    writeln!(cp_file, "Mach,CP_from_nose_m")?;

    // For CP, we pick a fixed time and shift it back.
    let fixed_time = Time::new::<second>(0.0);
    let mut state = get_initial_state(&config);
    state.time = fixed_time;

    let mut mesh = Mesh::default();
    mesh_generator.generate(&state, &config, &mut mesh);

    // Crucial step: compute surface properties after generation so solver works!
    mesh.compute_surface_properties();

    let cg_fixed = config.geometry.current_cg(fixed_time).z.get::<meter>();

    let mut solver = digital_twin_glue::prelude::AeroSolver::default();

    let alpha = 2.0_f64.to_radians();
    let altitude = 500.0;

    // Look up dynamically instead of using hardcoded sea-level Constants
    let (air_density, sos, dyn_viscosity) = digital_twin_glue::prelude::lookup_atmosphere(altitude);

    let mut mach = 0.1;
    while mach <= 3.01 {
        let v_mag = mach * sos;
        let v_x = v_mag * alpha.sin();
        let v_z = v_mag * alpha.cos();

        state.body_velocity = [
            Velocity::new::<meter_per_second>(v_x),
            Velocity::new::<meter_per_second>(0.0),
            Velocity::new::<meter_per_second>(v_z),
        ]
        .into();

        // Ensure altitude is realistic
        state.position.z = uom::si::f64::Length::new::<meter>(altitude);

        let out = solver.calculate_forces(&mesh, &state, air_density, sos, dyn_viscosity);

        let mut z_cp_mesh = 0.0;
        if out.force.x.abs() > 1e-6 {
            z_cp_mesh = out.torque.y / out.force.x;
        }

        let z_cp_geom = z_cp_mesh + cg_fixed;
        let cp_from_nose = body_length / 2.0 - z_cp_geom;

        writeln!(cp_file, "{:.2},{:.6}", mach, cp_from_nose)?;
        mach += 0.1;
    }

    println!("Done exporting center of pressure to cp_curve.csv.");
    Ok(())
}
