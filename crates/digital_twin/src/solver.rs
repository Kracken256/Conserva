use digital_twin_glue::prelude::*;
use nalgebra::Vector3;

/// A simplified standard atmosphere table.
/// Columns: (Altitude [m], Density [kg/m^3], Speed of Sound [m/s], Dynamic Viscosity [kg/(m·s)])
/// This table is used to estimate ambient flight conditions at a given altitude.
const ATMOSPHERE_TABLE: &[(f64, f64, f64, f64)] = &[
    (0.0, 1.225, 340.29, 1.789e-5),
    (2000.0, 1.007, 332.53, 1.726e-5),
    (4000.0, 0.819, 324.59, 1.661e-5),
    (6000.0, 0.660, 316.45, 1.595e-5),
    (8000.0, 0.525, 308.11, 1.527e-5),
    (10000.0, 0.413, 299.53, 1.458e-5),
    (15000.0, 0.194, 295.07, 1.422e-5),
    (20000.0, 0.088, 295.07, 1.422e-5),
    (30000.0, 0.018, 301.71, 1.475e-5),
    (40000.0, 0.004, 317.19, 1.601e-5),
    (50000.0, 0.001, 329.80, 1.704e-5),
    (60000.0, 0.0003, 314.66, 1.580e-5),
    (80000.0, 0.00001, 282.20, 1.320e-5),
];

/// Linearly interpolates the atmospheric properties based on altitude.
///
/// # Arguments
/// * `altitude` - The current altitude of the body in meters above sea level.
///
/// # Returns
/// A tuple containing `(density, speed_of_sound, dynamic_viscosity)`.
fn lookup_atmosphere(altitude: f64) -> (f64, f64, f64) {
    // Above 100km (Karman line), we assume negligible atmosphere.
    if altitude > 100_000.0 {
        return (1e-12, 280.0, 1.3e-5);
    }

    // Clamp altitude to 0 to prevent issues below sea level.
    let alt = altitude.max(0.0);

    for i in 0..ATMOSPHERE_TABLE.len() - 1 {
        let p0 = ATMOSPHERE_TABLE[i];
        let p1 = ATMOSPHERE_TABLE[i + 1];

        // Perform linear interpolation between the two bracketing altitude points
        if alt <= p1.0 {
            let t = (alt - p0.0) / (p1.0 - p0.0);
            return (
                p0.1 + t * (p1.1 - p0.1),
                p0.2 + t * (p1.2 - p0.2),
                p0.3 + t * (p1.3 - p0.3),
            );
        }
    }

    // If we exceed the highest table value but are Below 100km, return the top-most row's properties
    let last = ATMOSPHERE_TABLE.last().unwrap();
    (last.1, last.2, last.3)
}

/// The main aerodynamic solver struct, implementing various Mach-dependent flight regimes.
#[derive(Debug, Clone, Default)]
pub struct TheSolver {}

impl AeroSolver for TheSolver {
    /// Evaluates the aerodynamic forces acting on a 3D mesh based on current airspeed and altitude.
    /// It discretizes the total flow across localized triangles depending on the flight regime,
    /// integrating the resulting force and torque over the entirety of the object surface.
    fn calculate_forces(&mut self, mesh: &Mesh, state: &MissileState) -> SolverOutput {
        let altitude = state.position.z.value.abs();
        let (air_density, speed_of_sound, dyn_viscosity) = lookup_atmosphere(altitude);

        // Treat center of mass near origin natively for the local formulation
        let cm_local = Vector3::zeros();

        // Extract linear body velocity & angular velocity
        let v_inf = state.body_velocity.map(|v| v.value);
        let w = state.angular_velocity.map(|w| w.value);

        // Calculate the freestream speed and corresponding Mach number
        let v_inf_mag = v_inf.norm();
        let freestream_mach = v_inf_mag / speed_of_sound;

        // Arbitrary reference length (used to estimate Reynolds Number)
        let length_ref = 1.0;

        // Evaluate conditions based on distinct Mach regimes
        if freestream_mach < 0.8 {
            // == Subsonic Regime (Mach < 0.8) ==
            // Apply the Prandtl-Glauert compressibility correction rule.
            let pg_beta = (1.0 - freestream_mach * freestream_mach).sqrt().max(0.4);

            Self::integrate_mesh(
                mesh,
                cm_local,
                v_inf,
                w,
                speed_of_sound,
                air_density,
                dyn_viscosity,
                length_ref,
                freestream_mach,
                // Resulting C_p on the windward side
                |cos_theta, _| (2.0 * cos_theta) / pg_beta,
            )
        } else if freestream_mach < 1.2 {
            // == Transonic Regime (0.8 <= Mach < 1.2) ==
            // Linearly blend between subsonic compressibility and supersonic scaling
            // to avoid discontinuities as it breaks through the sound barrier.
            let mach_blend = (freestream_mach - 0.8) / 0.4;

            // Anchor values evaluated exactly at Mach 0.8 and Mach 1.2 boundaries
            let cp_sub_coef = 2.0 / (1.0 - 0.64_f64).sqrt(); // Mach 0.8
            let cp_sup_coef = 2.0 / (1.44_f64 - 1.0).sqrt(); // Mach 1.2

            let cp_blend = cp_sub_coef * (1.0 - mach_blend) + cp_sup_coef * mach_blend;

            Self::integrate_mesh(
                mesh,
                cm_local,
                v_inf,
                w,
                speed_of_sound,
                air_density,
                dyn_viscosity,
                length_ref,
                freestream_mach,
                |cos_theta, _| cp_blend * cos_theta,
            )
        } else if freestream_mach < 5.0 {
            // == Supersonic Regime (1.2 <= Mach < 5.0) ==
            // Employs Ackeret's Linear Supersonic Theory for estimating the pressure coefficient.
            let beta = (freestream_mach * freestream_mach - 1.0).sqrt();

            Self::integrate_mesh(
                mesh,
                cm_local,
                v_inf,
                w,
                speed_of_sound,
                air_density,
                dyn_viscosity,
                length_ref,
                freestream_mach,
                |cos_theta, _| (2.0 * cos_theta) / beta,
            )
        } else {
            // == Hypersonic Regime (Mach >= 5.0) ==
            // Combines Ackeret Theory (at lower limits) with Newtonian Impact Theory (high limits).
            // This models inelastic flow collisions commonly seen at extreme speeds.
            let blend = ((freestream_mach - 5.0) / 2.0).clamp(0.0, 1.0);

            // Fixed reference boundary evaluated at Mach 5
            let beta_5 = (25.0_f64 - 1.0).sqrt();

            Self::integrate_mesh(
                mesh,
                cm_local,
                v_inf,
                w,
                speed_of_sound,
                air_density,
                dyn_viscosity,
                length_ref,
                freestream_mach,
                move |cos_theta, _| {
                    // Newtonian impact approximation: 2 * sin^2(theta_flow)
                    let cp_newtonian = 2.0 * cos_theta * cos_theta;
                    let cp_ackeret = (2.0 * cos_theta) / beta_5;
                    cp_ackeret * (1.0 - blend) + cp_newtonian * blend
                },
            )
        }
    }
}

impl TheSolver {
    /// Iterates through all triangles on the given 3D `mesh`, computes the
    /// localized aerodynamic forces (pressure and viscous skin friction),
    /// and accumulates them into a net spatial force and torque around the CG.
    ///
    /// # Arguments
    /// * `mesh` - The triangulated surface representing the geometry.
    /// * `cm` - The location of the center of mass in local coordinates.
    /// * `v_inf` - The freestream linear velocity vector of the body relative to the air.
    /// * `w` - The body's angular velocity vector.
    /// * `speed_of_sound` - Local atmospheric speed of sound (m/s).
    /// * `air_density` - Local atmospheric density (kg/m^3).
    /// * `dyn_viscosity` - Local atmospheric dynamic viscosity.
    /// * `length_ref` - Reference length used in determining Reynolds number.
    /// * `freestream_mach` - Freestream Mach number.
    /// * `calc_windward_cp` - A callback closure defining how to compute the localized
    ///   pressure coefficient (`C_p`) given the specific flow regime bounds.
    #[inline(always)]
    fn integrate_mesh<F>(
        mesh: &Mesh,
        cm: Vector3<f64>,
        v_inf: Vector3<f64>,
        w: Vector3<f64>,
        speed_of_sound: f64,
        air_density: f64,
        dyn_viscosity: f64,
        length_ref: f64,
        freestream_mach: f64,
        calc_windward_cp: F,
    ) -> SolverOutput
    where
        F: Fn(f64, f64) -> f64,
    {
        let mut total_force = Vector3::zeros();
        let mut total_torque = Vector3::zeros();

        // Loop over each triangular geometry chunk
        for chunk in mesh.indices.chunks_exact(3) {
            // Retrieve vertices corresponding to index triplets
            let v0 = mesh.vertices[chunk[0] as usize];
            let v1 = mesh.vertices[chunk[1] as usize];
            let v2 = mesh.vertices[chunk[2] as usize];

            // Define edges intersecting at v0 to find the surface normal and area
            let edge1 = v1 - v0;
            let edge2 = v2 - v0;
            let cross = edge1.cross(&edge2);
            let area = cross.magnitude() * 0.5;

            // Cull degenerate triangles
            if area < 1e-8 {
                continue;
            }

            // Normal relative to the outbound facing mesh surface
            let normal = cross.normalize();

            // Geographic centroid of the individual triangle
            let centroid = (v0 + v1 + v2) / 3.0;

            // Offset vector to the Center of Mass (used for localized velocities + torque)
            let r = centroid - cm;

            // Overall localized geometric velocity combines translation and planar rotation
            let v_local = v_inf + w.cross(&r);
            let v_mag = v_local.norm();

            // Filter out stagnant points preventing zero-division errors
            if v_mag < 1e-3 {
                continue;
            }

            let local_mach = v_mag / speed_of_sound;

            // The air flow direction is the opposite of the body's local geometric velocity
            let v_dir = -v_local / v_mag;
            // Calculate incidence angle alignment metric
            // Note: because normals point outward, a negative dot product indicates
            // the airflow is striking the face directly (windward side).
            let cos_theta = normal.dot(&v_dir);

            // Determine local pressure coefficient (C_p)
            let c_p = if cos_theta < 0.0 {
                // WINDWARD
                calc_windward_cp(cos_theta.abs(), local_mach)
            } else {
                // LEEWARD (Base Drag / Wake)
                // We blend from subsonic separation (-0.1) to supersonic vacuum (-1/M^2)
                // across the transonic regime (0.8 to 1.2 Mach).
                if freestream_mach < 0.8 {
                    -0.1
                } else if freestream_mach < 1.2 {
                    let blend = (freestream_mach - 0.8) / 0.4;
                    let cp_sub = -0.1;
                    let cp_sup = -1.0 / (1.2 * 1.2);
                    cp_sub * (1.0 - blend) + cp_sup * blend
                } else {
                    // Supersonic base pressure approximation
                    // As Mach increases, base pressure approaches a vacuum (-1/gamma*M^2)
                    -1.0 / (freestream_mach * freestream_mach)
                }
            };

            // Calculate the boundary-layer distinct Reynolds number across the element
            let reynolds = (air_density * v_mag * length_ref) / dyn_viscosity;

            // Estimate skin friction coefficient (C_f).
            // Distinguishes between laminar (~0.002) and fully turbulent flat-plate models.
            let c_f = if reynolds > 1e5 {
                0.074 / reynolds.powf(0.2) // Prandtl’s 1/7th Power Law Approximation
            } else {
                0.002
            };

            // Dynamic pressure (q = 1/2 * rho * V^2)
            let q = 0.5 * air_density * v_mag * v_mag;

            // Resolve the force pushing inwardly perpendicular to the surface
            let force_normal = -normal * (q * c_p * area);

            // Calculate the velocity component wiping across the tangent plane
            let v_tangent = v_dir - normal * cos_theta;
            let v_tangent_mag = v_tangent.norm();

            // Compute sliding shear/friction opposing the planar movement direction
            let force_tangent = if v_tangent_mag > 1e-6 {
                let tangent_dir = v_tangent / v_tangent_mag;
                // v_dir is the airflow direction, so skin friction pulls the body IN the airflow direction.
                tangent_dir * (q * c_f * area)
            } else {
                Vector3::zeros()
            };

            // Total elemental force and moment accumulation applied to global body sum
            let force_vec = force_normal + force_tangent;
            total_force += force_vec;
            total_torque += r.cross(&force_vec);
        }

        SolverOutput {
            force: total_force,
            torque: total_torque,
        }
    }
}
