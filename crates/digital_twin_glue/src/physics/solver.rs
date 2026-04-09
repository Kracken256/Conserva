use nalgebra::Vector3;

pub struct SolverOutput {
    pub force: Vector3<f64>,
    pub torque: Vector3<f64>,
}

use crate::prelude::{Mesh, MissileState};

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
pub fn lookup_atmosphere(altitude: f64) -> (f64, f64, f64) {
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
pub struct AeroSolver {}

impl AeroSolver {
    /// Evaluates the aerodynamic forces acting on a 3D mesh based on current airspeed and altitude.
    /// It discretizes the total flow across localized triangles depending on the flight regime,
    /// integrating the resulting force and torque over the entirety of the object surface.
    /// This solver employs 8-wide SIMD vectors to maximize throughput.
    pub fn calculate_forces(
        &mut self,
        mesh: &Mesh,
        state: &MissileState,
        air_density: f64,
        speed_of_sound: f64,
        dyn_viscosity: f64,
    ) -> SolverOutput {
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
                |cos_theta, _| (cos_theta * 2.0) / f64x8::splat(pg_beta),
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
                |cos_theta, _| cos_theta * f64x8::splat(cp_blend),
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
                |cos_theta, _| (cos_theta * 2.0) / f64x8::splat(beta),
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
                    let cp_newtonian = cos_theta * cos_theta * f64x8::splat(2.0);
                    let cp_ackeret = (cos_theta * 2.0) / f64x8::splat(beta_5);
                    cp_ackeret * f64x8::splat(1.0 - blend) + cp_newtonian * f64x8::splat(blend)
                },
            )
        }
    }
}

use wide::{CmpGe, CmpGt, CmpLt, f64x8};

impl AeroSolver {
    /// Integrates the localized aerodynamic forces (pressure and viscous skin friction)
    /// across all triangles in the given 3D `mesh`, accumulating them into a net spatial
    /// force and torque around the CG. The integration uses `f64x8` SIMD batches for performance.
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
        F: Fn(f64x8, f64x8) -> f64x8,
    {
        let mut total_force = Vector3::zeros();
        let mut total_torque = Vector3::zeros();

        let leeward_cp = if freestream_mach < 0.8 {
            -0.1
        } else if freestream_mach < 1.2 {
            let blend = (freestream_mach - 0.8) / 0.4;
            let cp_sub = -0.1;
            let cp_sup = -1.0 / (1.2 * 1.2);
            cp_sub * (1.0 - blend) + cp_sup * blend
        } else {
            -1.0 / (freestream_mach * freestream_mach)
        };

        let n_faces = mesh.faces.area.len();
        let chunks = n_faces / 8;
        let remainder = n_faces % 8;

        let mut force_x = f64x8::splat(0.0);
        let mut force_y = f64x8::splat(0.0);
        let mut force_z = f64x8::splat(0.0);

        let mut torque_x = f64x8::splat(0.0);
        let mut torque_y = f64x8::splat(0.0);
        let mut torque_z = f64x8::splat(0.0);

        let v_inf_x = f64x8::splat(v_inf.x);
        let v_inf_y = f64x8::splat(v_inf.y);
        let v_inf_z = f64x8::splat(v_inf.z);

        let w_x = f64x8::splat(w.x);
        let w_y = f64x8::splat(w.y);
        let w_z = f64x8::splat(w.z);

        let cm_x = f64x8::splat(cm.x);
        let cm_y = f64x8::splat(cm.y);
        let cm_z = f64x8::splat(cm.z);

        let leeward_cp_splat = f64x8::splat(leeward_cp);
        let speed_of_sound_splat = f64x8::splat(speed_of_sound);
        let air_density_splat = f64x8::splat(air_density);
        let length_ref_splat = f64x8::splat(length_ref);
        let dyn_viscosity_splat = f64x8::splat(dyn_viscosity);

        // Helper for vector length/norm
        let norm3 =
            |x: f64x8, y: f64x8, z: f64x8| -> f64x8 { ((x * x) + (y * y) + (z * z)).sqrt() };

        let cross3 = |ax: f64x8,
                      ay: f64x8,
                      az: f64x8,
                      bx: f64x8,
                      by: f64x8,
                      bz: f64x8|
         -> (f64x8, f64x8, f64x8) {
            (
                (ay * bz) - (az * by),
                (az * bx) - (ax * bz),
                (ax * by) - (ay * bx),
            )
        };

        for c in 0..chunks {
            let offset = c * 8;

            let area_arr = [
                mesh.faces.area[offset],
                mesh.faces.area[offset + 1],
                mesh.faces.area[offset + 2],
                mesh.faces.area[offset + 3],
                mesh.faces.area[offset + 4],
                mesh.faces.area[offset + 5],
                mesh.faces.area[offset + 6],
                mesh.faces.area[offset + 7],
            ];
            let area = f64x8::new(area_arr);

            // Early bailout for degenerate
            let area_mask = area.simd_ge(f64x8::splat(1e-8));
            if area_mask.to_array() == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] {
                continue;
            }

            let cx = f64x8::new([
                mesh.faces.centroid_x[offset], mesh.faces.centroid_x[offset+1],
                mesh.faces.centroid_x[offset+2], mesh.faces.centroid_x[offset+3],
                mesh.faces.centroid_x[offset+4], mesh.faces.centroid_x[offset+5],
                mesh.faces.centroid_x[offset+6], mesh.faces.centroid_x[offset+7]
            ]);
            let cy = f64x8::new([
                mesh.faces.centroid_y[offset], mesh.faces.centroid_y[offset+1],
                mesh.faces.centroid_y[offset+2], mesh.faces.centroid_y[offset+3],
                mesh.faces.centroid_y[offset+4], mesh.faces.centroid_y[offset+5],
                mesh.faces.centroid_y[offset+6], mesh.faces.centroid_y[offset+7]
            ]);
            let cz = f64x8::new([
                mesh.faces.centroid_z[offset], mesh.faces.centroid_z[offset+1],
                mesh.faces.centroid_z[offset+2], mesh.faces.centroid_z[offset+3],
                mesh.faces.centroid_z[offset+4], mesh.faces.centroid_z[offset+5],
                mesh.faces.centroid_z[offset+6], mesh.faces.centroid_z[offset+7]
            ]);

            let rx = cx - cm_x;
            let ry = cy - cm_y;
            let rz = cz - cm_z;

            let (cross_x, cross_y, cross_z) = cross3(w_x, w_y, w_z, rx, ry, rz);
            let v_local_x = v_inf_x + cross_x;
            let v_local_y = v_inf_y + cross_y;
            let v_local_z = v_inf_z + cross_z;

            let v_mag = norm3(v_local_x, v_local_y, v_local_z);

            // Early bailout for stagnant points or fully degenerate chunks
            let v_mag_mask = v_mag.simd_ge(f64x8::splat(1e-3));
            let valid_mask = area_mask & v_mag_mask;
            if valid_mask.to_array() == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] {
                continue;
            }

            let safe_v_mag = v_mag.max(f64x8::splat(1e-3));
            let local_mach = safe_v_mag / speed_of_sound_splat;

            // v_dir = -v_local / v_mag
            let v_dir_x = (f64x8::splat(0.0) - v_local_x) / safe_v_mag;
            let v_dir_y = (f64x8::splat(0.0) - v_local_y) / safe_v_mag;
            let v_dir_z = (f64x8::splat(0.0) - v_local_z) / safe_v_mag;

            let nx = f64x8::new([
                mesh.faces.normal_x[offset], mesh.faces.normal_x[offset+1],
                mesh.faces.normal_x[offset+2], mesh.faces.normal_x[offset+3],
                mesh.faces.normal_x[offset+4], mesh.faces.normal_x[offset+5],
                mesh.faces.normal_x[offset+6], mesh.faces.normal_x[offset+7]
            ]);
            let ny = f64x8::new([
                mesh.faces.normal_y[offset], mesh.faces.normal_y[offset+1],
                mesh.faces.normal_y[offset+2], mesh.faces.normal_y[offset+3],
                mesh.faces.normal_y[offset+4], mesh.faces.normal_y[offset+5],
                mesh.faces.normal_y[offset+6], mesh.faces.normal_y[offset+7]
            ]);
            let nz = f64x8::new([
                mesh.faces.normal_z[offset], mesh.faces.normal_z[offset+1],
                mesh.faces.normal_z[offset+2], mesh.faces.normal_z[offset+3],
                mesh.faces.normal_z[offset+4], mesh.faces.normal_z[offset+5],
                mesh.faces.normal_z[offset+6], mesh.faces.normal_z[offset+7]
            ]);

            let cos_theta = (nx * v_dir_x) + (ny * v_dir_y) + (nz * v_dir_z);

            let windward_mask = cos_theta.simd_lt(f64x8::splat(0.0));
            // calc_windward_cp returns f64x8
            let cp_windward = calc_windward_cp(cos_theta.abs(), local_mach);
            let c_p = windward_mask.blend(cp_windward, leeward_cp_splat);

            let reynolds =
                (air_density_splat * safe_v_mag * length_ref_splat) / dyn_viscosity_splat;
            let turbulent_mask = reynolds.simd_gt(f64x8::splat(1e5));

            let safe_reynolds = reynolds.max(f64x8::splat(1e-5));

            // Calculate turbulent skin friction C_f using a 1/5th power law.
            // Since `wide` lacks a vectorized `powf` for `f64x8`, we fall back to a scalar array computation.
            // x^-0.2 is mathematically equivalent to 1.0 / x^0.2.
            let safe_rey_arr = safe_reynolds.to_array();
            let cp_turb_arr = [
                0.074 / safe_rey_arr[0].powf(0.2), 0.074 / safe_rey_arr[1].powf(0.2),
                0.074 / safe_rey_arr[2].powf(0.2), 0.074 / safe_rey_arr[3].powf(0.2),
                0.074 / safe_rey_arr[4].powf(0.2), 0.074 / safe_rey_arr[5].powf(0.2),
                0.074 / safe_rey_arr[6].powf(0.2), 0.074 / safe_rey_arr[7].powf(0.2)
            ];
            let cp_turb = f64x8::new(cp_turb_arr);

            let c_f = turbulent_mask.blend(cp_turb, f64x8::splat(0.002));

            let q = f64x8::splat(0.5) * air_density_splat * safe_v_mag * safe_v_mag;

            // normal force
            let fn_mag = q * c_p * area;
            let fn_x = (f64x8::splat(0.0) - nx) * fn_mag;
            let fn_y = (f64x8::splat(0.0) - ny) * fn_mag;
            let fn_z = (f64x8::splat(0.0) - nz) * fn_mag;

            // tangent velocity
            let vt_x = v_dir_x - (nx * cos_theta);
            let vt_y = v_dir_y - (ny * cos_theta);
            let vt_z = v_dir_z - (nz * cos_theta);
            let vt_mag = norm3(vt_x, vt_y, vt_z);

            let tangent_mask = vt_mag.simd_gt(f64x8::splat(1e-6));
            let safe_vt_mag = vt_mag.max(f64x8::splat(1e-6));
            let tangent_dir_x = vt_x / safe_vt_mag;
            let tangent_dir_y = vt_y / safe_vt_mag;
            let tangent_dir_z = vt_z / safe_vt_mag;

            let ft_mag = q * c_f * area;
            // Blend tangent force based on mask
            let ft_x = tangent_mask.blend(tangent_dir_x * ft_mag, f64x8::splat(0.0));
            let ft_y = tangent_mask.blend(tangent_dir_y * ft_mag, f64x8::splat(0.0));
            let ft_z = tangent_mask.blend(tangent_dir_z * ft_mag, f64x8::splat(0.0));

            // Accumulate total elemental forces.
            // The valid_mask ensures degenerate or stagnant faces realistically contribute exactly 0.0.
            let f_x = valid_mask.blend(fn_x + ft_x, f64x8::splat(0.0));
            let f_y = valid_mask.blend(fn_y + ft_y, f64x8::splat(0.0));
            let f_z = valid_mask.blend(fn_z + ft_z, f64x8::splat(0.0));

            force_x = force_x + f_x;
            force_y = force_y + f_y;
            force_z = force_z + f_z;

            let (t_x, t_y, t_z) = cross3(rx, ry, rz, f_x, f_y, f_z);
            torque_x = torque_x + t_x;
            torque_y = torque_y + t_y;
            torque_z = torque_z + t_z;
        }

        let f_arr_x = force_x.to_array();
        let f_arr_y = force_y.to_array();
        let f_arr_z = force_z.to_array();

        let t_arr_x = torque_x.to_array();
        let t_arr_y = torque_y.to_array();
        let t_arr_z = torque_z.to_array();

        total_force.x += f_arr_x[0] + f_arr_x[1] + f_arr_x[2] + f_arr_x[3] + f_arr_x[4] + f_arr_x[5] + f_arr_x[6] + f_arr_x[7];
        total_force.y += f_arr_y[0] + f_arr_y[1] + f_arr_y[2] + f_arr_y[3] + f_arr_y[4] + f_arr_y[5] + f_arr_y[6] + f_arr_y[7];
        total_force.z += f_arr_z[0] + f_arr_z[1] + f_arr_z[2] + f_arr_z[3] + f_arr_z[4] + f_arr_z[5] + f_arr_z[6] + f_arr_z[7];

        total_torque.x += t_arr_x[0] + t_arr_x[1] + t_arr_x[2] + t_arr_x[3] + t_arr_x[4] + t_arr_x[5] + t_arr_x[6] + t_arr_x[7];
        total_torque.y += t_arr_y[0] + t_arr_y[1] + t_arr_y[2] + t_arr_y[3] + t_arr_y[4] + t_arr_y[5] + t_arr_y[6] + t_arr_y[7];
        total_torque.z += t_arr_z[0] + t_arr_z[1] + t_arr_z[2] + t_arr_z[3] + t_arr_z[4] + t_arr_z[5] + t_arr_z[6] + t_arr_z[7];

        // Do the scalar loop for the remainder elements
        let rem_start = n_faces - remainder;
        for i in rem_start..n_faces {
            let area_scalar = mesh.faces.area[i];
            if area_scalar < 1e-8 {
                continue;
            }

            let r_scalar = Vector3::new(
                mesh.faces.centroid_x[i],
                mesh.faces.centroid_y[i],
                mesh.faces.centroid_z[i],
            ) - cm;
            let n_scalar = Vector3::new(
                mesh.faces.normal_x[i],
                mesh.faces.normal_y[i],
                mesh.faces.normal_z[i],
            );

            let loc_v = v_inf + w.cross(&r_scalar);
            let vm = loc_v.norm();
            if vm < 1e-3 {
                continue;
            }

            let lmach = vm / speed_of_sound;
            let d = -loc_v / vm;
            let cdt = n_scalar.dot(&d);

            // Broadcast scalar remainder variables into f64x8 vectors to satisfy the `calc_windward_cp` signature.
            let cw_arr = calc_windward_cp(f64x8::splat(cdt.abs()), f64x8::splat(lmach)).to_array();
            let cp_w = cw_arr[0];
            let cp = if cdt < 0.0 { cp_w } else { leeward_cp };

            let rey = (air_density * vm * length_ref) / dyn_viscosity;
            let cf = if rey > 1e5 {
                0.074 / rey.powf(0.2)
            } else {
                0.002
            };
            let qq = 0.5 * air_density * vm * vm;

            let fnrm = -n_scalar * (qq * cp * area_scalar);
            let vtan = d - n_scalar * cdt;
            let vtm = vtan.norm();
            let ftan = if vtm > 1e-6 {
                (vtan / vtm) * (qq * cf * area_scalar)
            } else {
                Vector3::zeros()
            };

            let f_el = fnrm + ftan;
            total_force += f_el;
            total_torque += r_scalar.cross(&f_el);
        }

        SolverOutput {
            force: total_force,
            torque: total_torque,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Matrix3, UnitQuaternion, Vector3};
    use uom::si::angle::radian;
    use uom::si::angular_velocity::radian_per_second;
    use uom::si::f64::*;
    use uom::si::force::newton;
    use uom::si::mass::kilogram;
    use uom::si::time::second;
    use uom::si::velocity::meter_per_second;

    fn create_dummy_state(v_inf: Vector3<f64>, w: Vector3<f64>) -> MissileState {
        MissileState {
            time: Time::new::<second>(0.0),
            position: Vector3::zeros(),
            body_velocity: Vector3::new(
                Velocity::new::<meter_per_second>(v_inf.x),
                Velocity::new::<meter_per_second>(v_inf.y),
                Velocity::new::<meter_per_second>(v_inf.z),
            ),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::new(
                AngularVelocity::new::<radian_per_second>(w.x),
                AngularVelocity::new::<radian_per_second>(w.y),
                AngularVelocity::new::<radian_per_second>(w.z),
            ),
            fin_angles: [Angle::new::<radian>(0.0); 4],
            tvc_angles: [Angle::new::<radian>(0.0); 2],
            current_mass: Mass::new::<kilogram>(10.0),
            motor_thrust: Force::new::<newton>(0.0),
            inertia_tensor: Matrix3::zeros(),
        }
    }

    fn create_faces(normals: &[Vector3<f64>], areas: &[f64], centroids: &[Vector3<f64>]) -> Mesh {
        let mut mesh = Mesh::default();
        for ((n, a), c) in normals.iter().zip(areas.iter()).zip(centroids.iter()) {
            mesh.faces.normal_x.push(n.x);
            mesh.faces.normal_y.push(n.y);
            mesh.faces.normal_z.push(n.z);
            mesh.faces.area.push(*a);
            mesh.faces.centroid_x.push(c.x);
            mesh.faces.centroid_y.push(c.y);
            mesh.faces.centroid_z.push(c.z);
        }
        mesh
    }

    fn assert_vec3_eq(a: Vector3<f64>, b: Vector3<f64>, tol: f64) {
        assert!((a.x - b.x).abs() < tol, "X mismatch: {} vs {}", a.x, b.x);
        assert!((a.y - b.y).abs() < tol, "Y mismatch: {} vs {}", a.y, b.y);
        assert!((a.z - b.z).abs() < tol, "Z mismatch: {} vs {}", a.z, b.z);
    }

    #[test]
    fn test_subsonic_force() {
        let mut solver = AeroSolver::default();
        let sos = 340.0;
        let v_z = 0.5 * sos; // Mach 0.5 -> Subsonic
        let state = create_dummy_state(Vector3::new(0.0, 0.0, v_z), Vector3::zeros());

        let n = Vector3::new(0.0, 0.0, 1.0); // Facing directly into the 'wind' (which is coming relative to +Z velocity, so air comes from +Z)
        let mesh = create_faces(&[n, n, n, n], &[1.0, 1.0, 1.0, 1.0], &[Vector3::zeros(); 4]);

        let out = solver.calculate_forces(&mesh, &state, 1.225, sos, 1.8e-5);

        // Windward coefficient:
        // beta = sqrt(1 - 0.5^2) = 0.866025
        // V_mag = 170.0
        // q = 0.5 * 1.225 * 170^2 = 17701.25
        // Cp = (1.0 * 2.0) / 0.866025 = 2.3094
        // Area = 4.0
        // Expected Force Z = q * Cp * Area = 17701.25 * 2.3094 * 4.0 = ~163533.15

        // Force is applied in opposite direction of normal (-Z)
        assert!(out.force.z < -100000.0);
        assert_vec3_eq(
            Vector3::new(out.force.x, out.force.y, 0.0),
            Vector3::zeros(),
            1e-4,
        );
        assert_vec3_eq(out.torque, Vector3::zeros(), 1e-4);
    }

    #[test]
    fn test_transonic_force() {
        let mut solver = AeroSolver::default();
        let sos = 340.0;
        let v_z = 1.0 * sos; // Mach 1.0 -> Transonic
        let state = create_dummy_state(Vector3::new(0.0, 0.0, v_z), Vector3::zeros());

        let n = Vector3::new(0.0, 0.0, 1.0);
        let mesh = create_faces(&[n, n, n, n], &[1.0, 1.0, 1.0, 1.0], &[Vector3::zeros(); 4]);

        let out = solver.calculate_forces(&mesh, &state, 1.225, sos, 1.8e-5);
        assert!(out.force.z < -200000.0); // Large transonic drag
    }

    #[test]
    fn test_supersonic_force() {
        let mut solver = AeroSolver::default();
        let sos = 340.0;
        let v_z = 3.0 * sos; // Mach 3.0
        let state = create_dummy_state(Vector3::new(0.0, 0.0, v_z), Vector3::zeros());

        let n = Vector3::new(0.0, 0.0, 1.0);
        let mesh = create_faces(&[n, n, n, n], &[1.0, 1.0, 1.0, 1.0], &[Vector3::zeros(); 4]);

        let out = solver.calculate_forces(&mesh, &state, 1.225, sos, 1.8e-5);

        // Supersonic Ackeret:
        // beta = sqrt(3^2 - 1) = sqrt(8) = 2.828
        // Cp = 2 / 2.828 = 0.707
        assert!(out.force.z < -100000.0);
    }

    #[test]
    fn test_hypersonic_force() {
        let mut solver = AeroSolver::default();
        let sos = 340.0;
        let v_z = 6.0 * sos; // Mach 6.0
        let state = create_dummy_state(Vector3::new(0.0, 0.0, v_z), Vector3::zeros());

        let n = Vector3::new(0.0, 0.0, 1.0);
        let mesh = create_faces(&[n, n, n, n], &[1.0, 1.0, 1.0, 1.0], &[Vector3::zeros(); 4]);

        let out = solver.calculate_forces(&mesh, &state, 1.225, sos, 1.8e-5);
        assert!(out.force.z < -1000000.0); // Very large hypersonic drag
    }

    #[test]
    fn test_degenerate_faces_ignored() {
        let mut solver = AeroSolver::default();
        let sos = 340.0;
        let state = create_dummy_state(Vector3::new(0.0, 0.0, sos), Vector3::zeros());

        // One valid face, 3 tiny degenerate faces
        let n = Vector3::new(0.0, 0.0, 1.0);
        let mesh = create_faces(
            &[n, n, n, n],
            &[1.0, 1e-10, 1e-10, 1e-10],
            &[Vector3::zeros(); 4],
        );
        let out_degenerate = solver.calculate_forces(&mesh, &state, 1.225, sos, 1.8e-5);

        // One valid face, 3 missing faces
        let mesh2 = create_faces(&[n], &[1.0], &[Vector3::zeros()]);
        let out_single = solver.calculate_forces(&mesh2, &state, 1.225, sos, 1.8e-5);

        assert_vec3_eq(out_degenerate.force, out_single.force, 1e-5);
    }

    #[test]
    fn test_stagnant_bailout() {
        let mut solver = AeroSolver::default();
        let sos = 340.0;
        // Near-zero velocity
        let state = create_dummy_state(Vector3::new(0.0, 0.0, 1e-4), Vector3::zeros());
        let n = Vector3::new(0.0, 0.0, 1.0);
        let mesh = create_faces(&[n, n, n, n], &[1.0, 1.0, 1.0, 1.0], &[Vector3::zeros(); 4]);

        let out = solver.calculate_forces(&mesh, &state, 1.225, sos, 1.8e-5);
        // Force should be completely zero because v < 1e-3 triggers early bailout
        assert_vec3_eq(out.force, Vector3::zeros(), 1e-10);
    }

    #[test]
    fn test_simd_vs_scalar_remainder() {
        let mut solver = AeroSolver::default();
        let sos = 340.0;
        let state = create_dummy_state(Vector3::new(0.0, 0.0, sos * 2.0), Vector3::zeros());

        let n = Vector3::new(0.0, 0.0, 1.0);
        let c = Vector3::zeros();

        // 8 faces (uses exactly 1 SIMD chunk)
        let mesh8 = create_faces(&[n, n, n, n, n, n, n, n], &[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0], &[c, c, c, c, c, c, c, c]);
        let out8 = solver.calculate_forces(&mesh8, &state, 1.225, sos, 1.8e-5);

        // 7 faces (uses exactly 7 scalar remainder, 0 SIMD)
        let mesh7 = create_faces(&[n, n, n, n, n, n, n], &[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0], &[c, c, c, c, c, c, c]);
        let out7 = solver.calculate_forces(&mesh7, &state, 1.225, sos, 1.8e-5);

        // 15 faces (1 SIMD chunk + 7 scalar remainder)
        let mesh15 = create_faces(
            &[n, n, n, n, n, n, n, n, n, n, n, n, n, n, n],
            &[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            &[c, c, c, c, c, c, c, c, c, c, c, c, c, c, c],
        );
        let out15 = solver.calculate_forces(&mesh15, &state, 1.225, sos, 1.8e-5);

        // We know that exactly proportional elements should yield proportional additive forces.
        let force_per_face = out8.force / 8.0;
        assert_vec3_eq(out7.force, force_per_face * 7.0, 1e-2);
        assert_vec3_eq(out15.force, force_per_face * 15.0, 1e-2);
    }

    #[test]
    fn test_angular_velocity() {
        let mut solver = AeroSolver::default();
        let sos = 340.0;

        // Zero linear velocity, pure pitch angular velocity
        let w = Vector3::new(10.0, 0.0, 0.0); // 10 rad/s pitch
        let state = create_dummy_state(Vector3::zeros(), w);

        // Two faces offset across the Y axis
        // R1 = (0, 1, 0), v_loc = w x R1 = (10,0,0) x (0,1,0) = (0,0,10)
        let mut mesh = Mesh::default();
        mesh.faces.normal_x.push(0.0);
        mesh.faces.normal_y.push(0.0);
        mesh.faces.normal_z.push(1.0); // faces +Z
        mesh.faces.area.push(1.0);
        mesh.faces.centroid_x.push(0.0);
        mesh.faces.centroid_y.push(1.0);
        mesh.faces.centroid_z.push(0.0); // +Y offset

        // R2 = (0, -1, 0), v_loc = w x R2 = (0,0,-10)
        mesh.faces.normal_x.push(0.0);
        mesh.faces.normal_y.push(0.0);
        mesh.faces.normal_z.push(-1.0); // faces -Z
        mesh.faces.area.push(1.0);
        mesh.faces.centroid_x.push(0.0);
        mesh.faces.centroid_y.push(-1.0);
        mesh.faces.centroid_z.push(0.0); // -Y offset

        let out = solver.calculate_forces(&mesh, &state, 1.225, sos, 1.8e-5);

        // A pure pitch acting on fins creates opposing drag loads that generate a pure damping torque on X
        assert!(out.torque.x < -100.0); // It asserts a damping torque opposite to the angular velocity (10.0)
        assert_vec3_eq(
            Vector3::new(0.0, out.torque.y, out.torque.z),
            Vector3::zeros(),
            1e-4,
        );
    }
}
