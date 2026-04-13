use crate::prelude::*;
use nalgebra::Vector3;
use wide::{CmpGe, CmpGt, CmpLt, f64x8};

pub struct SolverOutput {
    pub force: Vector3<f64>,
    pub torque: Vector3<f64>,
}

/// Core parameters delineating the distinct atmospheric boundary layers utilized
/// by the US Standard Atmosphere (ISA) profile.
/// Columns: (Base Altitude [m], Base Temperature [K], Lapse Rate [K/m], Base Pressure [Pa])
const ISA_LAYERS: &[(f64, f64, f64, f64)] = &[
    (0.0, 288.15, -0.0065, 101325.0),
    (11000.0, 216.65, 0.0, 22632.1),
    (20000.0, 216.65, 0.001, 5474.89),
    (32000.0, 228.65, 0.0028, 868.019),
    (47000.0, 270.65, 0.0, 110.906),
    (51000.0, 270.65, -0.0028, 66.9389),
    (71000.0, 214.65, -0.002, 3.95642),
    (86000.0, 186.87, 0.0, 0.3016),
];

/// Computes the ambient atmospheric properties utilizing the International Standard Atmosphere (ISA) model.
/// Provides continuous profiles for thermodynamic variables integrating standard thermocline layers mapping smoothly
/// up to the 86km mesopause limits.
///
/// # Arguments
/// * `altitude` - The current altitude of the body in meters above sea level.
///
/// # Returns
/// A tuple containing `(density [kg/m^3], speed_of_sound [m/s], dynamic_viscosity [kg/(m·s)])`.
pub fn lookup_atmosphere(altitude: f64) -> (f64, f64, f64) {
    if altitude > 100_000.0 {
        // Space approximation bounds
        return (1e-12, 280.0, 1.3e-5);
    }

    let alt = altitude.clamp(0.0, 86000.0);

    let mut layer = 0;
    for (i, _) in ISA_LAYERS.iter().enumerate().skip(1) {
        if alt < ISA_LAYERS[i].0 {
            break;
        }
        layer = i;
    }

    let (h_b, t_b, l_b, p_b) = ISA_LAYERS[layer];

    let g0 = 9.80665;
    let m_air = 0.0289644;
    let r_gas = 8.3144598;
    let gamma = 1.4;

    let t = (t_b + l_b * (alt - h_b)).max(1.0);
    let p = if l_b.abs() > 1e-10 {
        p_b * (t_b / t).powf((g0 * m_air) / (r_gas * l_b))
    } else {
        p_b * (-g0 * m_air * (alt - h_b) / (r_gas * t_b)).exp()
    };

    let density = (p * m_air) / (r_gas * t);
    let speed_of_sound = (gamma * (r_gas / m_air) * t).sqrt();

    // Sutherland's law for dynamic viscosity
    let mu0 = 1.716e-5;
    let t0 = 273.15;
    let c = 110.4;
    let dynamic_viscosity = mu0 * ((t0 + c) / (t + c)) * (t / t0).powf(1.5);

    (density, speed_of_sound, dynamic_viscosity)
}

#[derive(Debug, Clone)]
pub struct FreestreamEnv {
    pub cm: Vector3<f64>,
    pub v_inf: Vector3<f64>,
    pub w: Vector3<f64>,
    pub speed_of_sound: f64,
    pub air_density: f64,
    pub dyn_viscosity: f64,
    pub length_ref: f64,
    pub freestream_mach: f64,
}

/// Evaluates the aerodynamic forces acting on a 3D mesh based on current airspeed and altitude.
/// It discretizes the total flow across localized triangles depending on the flight regime,
/// integrating the resulting force and torque over the entirety of the object surface.
/// This solver employs 8-wide SIMD vectors to maximize throughput.
pub fn calculate_forces(
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

    let env = FreestreamEnv {
        cm: cm_local,
        v_inf,
        w,
        speed_of_sound,
        air_density,
        dyn_viscosity,
        length_ref,
        freestream_mach,
    };

    // Evaluate conditions based on distinct Mach regimes
    if freestream_mach < 0.8 {
        // == Subsonic Regime (Mach < 0.8) ==
        // Apply the Prandtl-Glauert compressibility correction rule.
        integrate_mesh(
            mesh,
            &env,
            // Resulting C_p on the windward side evaluated using local mach
            |cos_theta, local_mach| {
                let pg_beta = (f64x8::splat(1.0) - (local_mach * local_mach))
                    .max(f64x8::splat(0.16))
                    .sqrt();
                (cos_theta * f64x8::splat(2.0)) / pg_beta
            },
            |cos_theta, local_mach| {
                let pg_beta = (1.0 - local_mach * local_mach).max(0.16).sqrt();
                (cos_theta * 2.0) / pg_beta
            },
        )
    } else if freestream_mach < 1.2 {
        // == Transonic Regime (0.8 <= Mach < 1.2) ==
        // Linearly blend between subsonic compressibility and supersonic scaling
        // to avoid discontinuities as it breaks through the sound barrier.
        integrate_mesh(
            mesh,
            &env,
            |cos_theta, local_mach| {
                let mach_blend = (local_mach - f64x8::splat(0.8)) / f64x8::splat(0.4);
                let cp_sub_coef = f64x8::splat(2.0 / (1.0 - 0.8_f64.powi(2)).sqrt());
                let cp_sup_coef = f64x8::splat(2.0 / (1.2_f64.powi(2) - 1.0).sqrt());
                let cp_blend =
                    cp_sub_coef * (f64x8::splat(1.0) - mach_blend) + cp_sup_coef * mach_blend;
                cos_theta * cp_blend
            },
            |cos_theta, local_mach| {
                let mach_blend = (local_mach - 0.8) / 0.4;
                let cp_sub_coef = 2.0 / (1.0 - 0.8_f64.powi(2)).sqrt();
                let cp_sup_coef = 2.0 / (1.2_f64.powi(2) - 1.0).sqrt();
                let cp_blend = cp_sub_coef * (1.0 - mach_blend) + cp_sup_coef * mach_blend;
                cos_theta * cp_blend
            },
        )
    } else if freestream_mach < 5.0 {
        // == Supersonic Regime (1.2 <= Mach < 5.0) ==
        // Employs Ackeret's Linear Supersonic Theory for estimating the pressure coefficient.
        integrate_mesh(
            mesh,
            &env,
            |cos_theta, local_mach| {
                let beta = (local_mach * local_mach - f64x8::splat(1.0))
                    .max(f64x8::splat(0.0))
                    .sqrt();
                let safe_beta = beta.max(f64x8::splat(0.01));
                (cos_theta * f64x8::splat(2.0)) / safe_beta
            },
            |cos_theta, local_mach| {
                let beta = (local_mach * local_mach - 1.0).max(0.0).sqrt().max(0.01);
                (cos_theta * 2.0) / beta
            },
        )
    } else {
        // == Hypersonic Regime (Mach >= 5.0) ==
        // Combines Ackeret Theory (at lower limits) with Newtonian Impact Theory (high limits).
        // This models inelastic flow collisions commonly seen at extreme speeds.
        integrate_mesh(
            mesh,
            &env,
            |cos_theta, local_mach| {
                // Safe clamp interpolation
                let upper = local_mach.min(f64x8::splat(7.0));
                let blend =
                    ((upper - f64x8::splat(5.0)) / f64x8::splat(2.0)).max(f64x8::splat(0.0));

                let beta_5 = f64x8::splat((25.0_f64 - 1.0).sqrt());

                // Account for chemical dissociation at high Mach numbers (real gas effects)
                // For ideal gas (gamma=1.4), Newtonian Cp_max asymptotically approaches ~1.839.
                // Above Mach 8 (non-negligible dissociation), bonds break, lowering the
                // effective gamma, pushing Cp_max upwards toward ~1.93.
                let dissociation_effect = ((local_mach - f64x8::splat(8.0)) / f64x8::splat(12.0))
                    .max(f64x8::splat(0.0))
                    .min(f64x8::splat(1.0));
                let cp_max = f64x8::splat(1.839) + (f64x8::splat(0.091) * dissociation_effect);

                let cp_newtonian = cos_theta * cos_theta * cp_max;
                let cp_ackeret = (cos_theta * f64x8::splat(2.0)) / beta_5;
                cp_ackeret * (f64x8::splat(1.0) - blend) + cp_newtonian * blend
            },
            |cos_theta, local_mach| {
                let blend = ((local_mach - 5.0) / 2.0).clamp(0.0, 1.0);
                let beta_5 = (25.0_f64 - 1.0).sqrt();

                let dissociation_effect = ((local_mach - 8.0) / 12.0).clamp(0.0, 1.0);
                let cp_max = 1.839 + 0.091 * dissociation_effect;

                let cp_newtonian = cos_theta * cos_theta * cp_max;
                let cp_ackeret = (cos_theta * 2.0) / beta_5;
                cp_ackeret * (1.0 - blend) + cp_newtonian * blend
            },
        )
    }
}

/// Computes a fast approximation of x^(-0.2) which is used for the
/// 1/5th power law of turbulent boundary layer skin friction.
/// Uses a 64-bit magic constant adapted for inverse 5th root,
/// scaled for Reynolds number calculations.
#[inline(always)]
fn fast_inv_fifth_root(x: f64) -> f64 {
    let i = x.to_bits();
    let j = 0x4cb8c612284ba400_u64 - i / 5;
    let y = f64::from_bits(j);
    let y2 = y * y;
    let y4 = y2 * y2;
    let y5 = y4 * y;
    // Single Newton-Raphson iteration
    y * (1.2 - 0.2 * x * y5)
}

#[inline(always)]
fn norm3(x: f64x8, y: f64x8, z: f64x8) -> f64x8 {
    ((x * x) + (y * y) + (z * z)).sqrt()
}

#[inline(always)]
fn cross3(
    ax: f64x8,
    ay: f64x8,
    az: f64x8,
    bx: f64x8,
    by: f64x8,
    bz: f64x8,
) -> (f64x8, f64x8, f64x8) {
    (
        (ay * bz) - (az * by),
        (az * bx) - (ax * bz),
        (ax * by) - (ay * bx),
    )
}

#[inline(always)]
fn load_chunk(arr: &[f64], offset: usize) -> f64x8 {
    f64x8::new(arr[offset..offset + 8].try_into().unwrap())
}

#[inline(always)]
fn map_f64x8(x: f64x8, f: impl Fn(f64) -> f64) -> f64x8 {
    let arr = x.to_array();
    f64x8::new([
        f(arr[0]),
        f(arr[1]),
        f(arr[2]),
        f(arr[3]),
        f(arr[4]),
        f(arr[5]),
        f(arr[6]),
        f(arr[7]),
    ])
}

fn get_leeward_cp(mach: f64) -> f64 {
    if mach < 0.8 {
        -0.1
    } else if mach < 1.2 {
        let blend = (mach - 0.8) / 0.4;
        let cp_sub = -0.1;
        let cp_sup = -1.0 / (1.2 * 1.2);
        cp_sub * (1.0 - blend) + cp_sup * blend
    } else {
        -1.0 / (mach * mach)
    }
}

fn integrate_simd_chunks<F>(
    mesh: &Mesh,
    env: &FreestreamEnv,
    leeward_cp: f64,
    chunks: usize,
    calc_windward_cp: &F,
) -> (Vector3<f64>, Vector3<f64>)
where
    F: Fn(f64x8, f64x8) -> f64x8,
{
    let mut force_x = f64x8::splat(0.0);
    let mut force_y = f64x8::splat(0.0);
    let mut force_z = f64x8::splat(0.0);

    let mut torque_x = f64x8::splat(0.0);
    let mut torque_y = f64x8::splat(0.0);
    let mut torque_z = f64x8::splat(0.0);

    let v_inf_x = f64x8::splat(env.v_inf.x);
    let v_inf_y = f64x8::splat(env.v_inf.y);
    let v_inf_z = f64x8::splat(env.v_inf.z);

    let w_x = f64x8::splat(env.w.x);
    let w_y = f64x8::splat(env.w.y);
    let w_z = f64x8::splat(env.w.z);

    let cm_x = f64x8::splat(env.cm.x);
    let cm_y = f64x8::splat(env.cm.y);
    let cm_z = f64x8::splat(env.cm.z);

    let leeward_cp_splat = f64x8::splat(leeward_cp);
    let speed_of_sound_splat = f64x8::splat(env.speed_of_sound);
    let air_density_splat = f64x8::splat(env.air_density);
    let length_ref_splat = f64x8::splat(env.length_ref);
    let dyn_viscosity_splat = f64x8::splat(env.dyn_viscosity);

    for c in 0..chunks {
        let offset = c * 8;

        let area = load_chunk(&mesh.faces.area, offset);

        // Early bailout for degenerate
        let area_mask = area.simd_ge(f64x8::splat(1e-8));
        if area_mask.to_array() == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] {
            continue;
        }

        let cx = load_chunk(&mesh.faces.centroid_x, offset);
        let cy = load_chunk(&mesh.faces.centroid_y, offset);
        let cz = load_chunk(&mesh.faces.centroid_z, offset);

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

        let nx = load_chunk(&mesh.faces.normal_x, offset);
        let ny = load_chunk(&mesh.faces.normal_y, offset);
        let nz = load_chunk(&mesh.faces.normal_z, offset);

        let cos_theta = (nx * v_dir_x) + (ny * v_dir_y) + (nz * v_dir_z);

        let windward_mask = cos_theta.simd_lt(f64x8::splat(0.0));
        // calc_windward_cp returns f64x8
        let cp_windward = calc_windward_cp(cos_theta.abs(), local_mach);
        let c_p = windward_mask.blend(cp_windward, leeward_cp_splat);

        let reynolds = (air_density_splat * safe_v_mag * length_ref_splat) / dyn_viscosity_splat;
        let turbulent_mask = reynolds.simd_gt(f64x8::splat(1e5));

        let safe_reynolds = reynolds.max(f64x8::splat(1e-5));
        let knudsen = f64x8::splat(1.482) * local_mach / safe_reynolds;
        let slip_correction = f64x8::splat(1.0) / (f64x8::splat(1.0) + knudsen);

        // Calculate turbulent skin friction C_f using a 1/5th power law.
        // Since `wide` lacks a vectorized `powf` for `f64x8`, we fall back to a scalar array computation.
        // x^-0.2 is mathematically equivalent to 1.0 / x^0.2.
        let cp_turb = map_f64x8(safe_reynolds, |r| 0.074 * fast_inv_fifth_root(r));

        let c_f = turbulent_mask.blend(cp_turb, f64x8::splat(0.002)) * slip_correction;

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

        force_x += f_x;
        force_y += f_y;
        force_z += f_z;

        let (t_x, t_y, t_z) = cross3(rx, ry, rz, f_x, f_y, f_z);
        torque_x += t_x;
        torque_y += t_y;
        torque_z += t_z;
    }

    let f_arr_x = force_x.to_array();
    let f_arr_y = force_y.to_array();
    let f_arr_z = force_z.to_array();

    let t_arr_x = torque_x.to_array();
    let t_arr_y = torque_y.to_array();
    let t_arr_z = torque_z.to_array();

    let total_force = Vector3::new(
        f_arr_x.iter().sum::<f64>(),
        f_arr_y.iter().sum::<f64>(),
        f_arr_z.iter().sum::<f64>(),
    );

    let total_torque = Vector3::new(
        t_arr_x.iter().sum::<f64>(),
        t_arr_y.iter().sum::<f64>(),
        t_arr_z.iter().sum::<f64>(),
    );

    (total_force, total_torque)
}

fn integrate_scalar_remainder<FS>(
    mesh: &Mesh,
    env: &FreestreamEnv,
    leeward_cp: f64,
    remainder_start: usize,
    n_faces: usize,
    calc_windward_cp_scalar: &FS,
) -> (Vector3<f64>, Vector3<f64>)
where
    FS: Fn(f64, f64) -> f64,
{
    let mut total_force = Vector3::zeros();
    let mut total_torque = Vector3::zeros();

    for i in remainder_start..n_faces {
        let area_scalar = mesh.faces.area[i];
        if area_scalar < 1e-8 {
            continue;
        }

        let r_scalar = Vector3::new(
            mesh.faces.centroid_x[i],
            mesh.faces.centroid_y[i],
            mesh.faces.centroid_z[i],
        ) - env.cm;
        let n_scalar = Vector3::new(
            mesh.faces.normal_x[i],
            mesh.faces.normal_y[i],
            mesh.faces.normal_z[i],
        );

        let loc_v = env.v_inf + env.w.cross(&r_scalar);
        let vm = loc_v.norm();
        if vm < 1e-3 {
            continue;
        }

        let lmach = vm / env.speed_of_sound;
        let d = -loc_v / vm;
        let cdt = n_scalar.dot(&d);

        // Directly evaluate the scalar windward cp without SIMD splatting overhead.
        let cp_w = calc_windward_cp_scalar(cdt.abs(), lmach);
        let cp = if cdt < 0.0 { cp_w } else { leeward_cp };

        let rey = (env.air_density * vm * env.length_ref) / env.dyn_viscosity;
        let knudsen = 1.482 * lmach / rey.max(1e-5);
        let slip_correction = 1.0 / (1.0 + knudsen);

        let cf = if rey > 1e5 {
            0.074 * fast_inv_fifth_root(rey)
        } else {
            0.002
        } * slip_correction;
        let qq = 0.5 * env.air_density * vm * vm;

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

    (total_force, total_torque)
}

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
fn integrate_mesh<F, FS>(
    mesh: &Mesh,
    env: &FreestreamEnv,
    calc_windward_cp: F,
    calc_windward_cp_scalar: FS,
) -> SolverOutput
where
    F: Fn(f64x8, f64x8) -> f64x8,
    FS: Fn(f64, f64) -> f64,
{
    let leeward_cp = get_leeward_cp(env.freestream_mach);

    let n_faces = mesh.faces.area.len();
    let chunks = n_faces / 8;
    let remainder = n_faces % 8;

    let (simd_force, simd_torque) =
        integrate_simd_chunks(mesh, env, leeward_cp, chunks, &calc_windward_cp);

    let rem_start = n_faces - remainder;
    let (rem_force, rem_torque) = integrate_scalar_remainder(
        mesh,
        env,
        leeward_cp,
        rem_start,
        n_faces,
        &calc_windward_cp_scalar,
    );

    SolverOutput {
        force: simd_force + rem_force,
        torque: simd_torque + rem_torque,
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

    fn assert_approx_eq(a: f64, b: f64, tol: f64) {
        assert!(
            (a - b).abs() < tol,
            "Expected ~{}, got {} (diff {})",
            b,
            a,
            (a - b).abs()
        );
    }

    #[test]
    fn test_atmosphere_model_sea_level() {
        let (rho, sos, dyn_visc) = lookup_atmosphere(0.0);
        assert_approx_eq(rho, 1.225, 0.001);
        assert_approx_eq(sos, 340.297, 0.01);
        assert_approx_eq(dyn_visc, 1.789e-5, 1e-8);
    }

    #[test]
    fn test_subsonic_force_exact() {
        let sos = 340.0;
        let v_z = 0.5 * sos; // Mach 0.5 -> Subsonic
        let state = create_dummy_state(Vector3::new(0.0, 0.0, v_z), Vector3::zeros());

        let n = Vector3::new(0.0, 0.0, 1.0); // Facing directly into the '+Z' wind
        let mesh = create_faces(&[n, n, n, n], &[1.0, 1.0, 1.0, 1.0], &[Vector3::zeros(); 4]);

        let out = crate::physics::calculate_forces(&mesh, &state, 1.225, sos, 1.8e-5);

        // Windward coefficient (Mach 0.5):
        // beta = sqrt(1 - 0.5^2) = 0.866025
        // V_mag = 170.0
        // q = 0.5 * 1.225 * 170^2 = 17701.25
        // Cp = (1.0 * 2.0) / 0.866025 = 2.309401
        // Area = 4.0
        // Expected Force Z = -q * Cp * Area = -163517.14
        assert_approx_eq(out.force.z, -163517.14, 1.0);
        assert_vec3_eq(
            Vector3::new(out.force.x, out.force.y, 0.0),
            Vector3::zeros(),
            1e-4,
        );
        assert_vec3_eq(out.torque, Vector3::zeros(), 1e-4);
    }

    #[test]
    fn test_transonic_force_exact() {
        let sos = 340.0;
        let v_z = 1.0 * sos; // Mach 1.0 -> exactly middle of Transonic blend
        let state = create_dummy_state(Vector3::new(0.0, 0.0, v_z), Vector3::zeros());

        let n = Vector3::new(0.0, 0.0, 1.0);
        let mesh = create_faces(&[n, n, n, n], &[1.0, 1.0, 1.0, 1.0], &[Vector3::zeros(); 4]);

        let out = crate::physics::calculate_forces(&mesh, &state, 1.225, sos, 1.8e-5);

        // Exactly Mach 1.0 translates to mach_blend = 0.5
        // cp_sub = 2 / sqrt(1 - 0.64) = 3.3333...
        // cp_sup = 2 / sqrt(1.44 - 1) = 3.015113...
        // Cp_blend = 3.1742...
        // q = 0.5 * 1.225 * 340^2 = 70805.0
        // Fz = -70805.0 * 3.1742 * 4.0 = -899003.54
        assert_approx_eq(out.force.z, -899003.54, 2.0);
    }

    #[test]
    fn test_lateral_aerodynamics_angle_of_attack() {
        let sos = 340.0;

        // V = 100 in X, 100 in Z
        let state = create_dummy_state(Vector3::new(100.0, 0.0, 100.0), Vector3::zeros());
        let n = Vector3::new(0.0, 0.0, 1.0); // Flat plate facing +Z
        let mesh = create_faces(&[n], &[1.0], &[Vector3::zeros()]);

        let out = crate::physics::calculate_forces(&mesh, &state, 1.225, sos, 1.8e-5);

        // Exact math constraints:
        // normal force isolates only the compression component facing exactly into the oncoming Z stream.
        // The plate generates exactly Fz = -19050.26 from normal forces.
        assert_approx_eq(out.force.z, -19050.26, 1.0);

        // Lateral velocity vt causes skin friction pulling tangentially in -X direction.
        // It must be negative, modeling drag rubbing against the sliding flow.
        assert!(out.force.x < -0.0);
        assert_approx_eq(out.force.y, 0.0, 1e-6);
    }

    #[test]
    fn test_supersonic_force() {
        let sos = 340.0;
        let v_z = 3.0 * sos; // Mach 3.0
        let state = create_dummy_state(Vector3::new(0.0, 0.0, v_z), Vector3::zeros());

        let n = Vector3::new(0.0, 0.0, 1.0);
        let mesh = create_faces(&[n, n, n, n], &[1.0, 1.0, 1.0, 1.0], &[Vector3::zeros(); 4]);

        let out = crate::physics::calculate_forces(&mesh, &state, 1.225, sos, 1.8e-5);

        // Supersonic Ackeret:
        // beta = sqrt(3^2 - 1) = sqrt(8) = 2.828427
        // Cp = 2 / 2.828427 = 0.707106
        // q = 0.5 * 1.225 * 1020^2 = 637245.0
        // Expected Force Z = -q * Cp * Area = -1802401.04
        assert_approx_eq(out.force.z, -1802401.04, 2.0);
    }

    #[test]
    fn test_hypersonic_force() {
        let sos = 340.0;
        let v_z = 6.0 * sos; // Mach 6.0
        let state = create_dummy_state(Vector3::new(0.0, 0.0, v_z), Vector3::zeros());

        let n = Vector3::new(0.0, 0.0, 1.0);
        let mesh = create_faces(&[n, n, n, n], &[1.0, 1.0, 1.0, 1.0], &[Vector3::zeros(); 4]);

        let out = crate::physics::calculate_forces(&mesh, &state, 1.225, sos, 1.8e-5);

        // Expected Force Z with real gas Newtonian factor
        assert_approx_eq(out.force.z, -11456381.89, 2.0);
    }

    #[test]
    fn test_degenerate_faces_ignored() {
        let sos = 340.0;
        let state = create_dummy_state(Vector3::new(0.0, 0.0, sos), Vector3::zeros());

        // One valid face, 3 tiny degenerate faces
        let n = Vector3::new(0.0, 0.0, 1.0);
        let mesh = create_faces(
            &[n, n, n, n],
            &[1.0, 1e-10, 1e-10, 1e-10],
            &[Vector3::zeros(); 4],
        );
        let out_degenerate = crate::physics::calculate_forces(&mesh, &state, 1.225, sos, 1.8e-5);

        // One valid face, 3 missing faces
        let mesh2 = create_faces(&[n], &[1.0], &[Vector3::zeros()]);
        let out_single = crate::physics::calculate_forces(&mesh2, &state, 1.225, sos, 1.8e-5);

        assert_vec3_eq(out_degenerate.force, out_single.force, 1e-5);
    }

    #[test]
    fn test_stagnant_bailout() {
        let sos = 340.0;
        // Near-zero velocity
        let state = create_dummy_state(Vector3::new(0.0, 0.0, 1e-4), Vector3::zeros());
        let n = Vector3::new(0.0, 0.0, 1.0);
        let mesh = create_faces(&[n, n, n, n], &[1.0, 1.0, 1.0, 1.0], &[Vector3::zeros(); 4]);

        let out = crate::physics::calculate_forces(&mesh, &state, 1.225, sos, 1.8e-5);
        // Force should be completely zero because v < 1e-3 triggers early bailout
        assert_vec3_eq(out.force, Vector3::zeros(), 1e-10);
    }

    #[test]
    fn test_simd_vs_scalar_remainder() {
        let sos = 340.0;
        let state = create_dummy_state(Vector3::new(0.0, 0.0, sos * 2.0), Vector3::zeros());

        let n = Vector3::new(0.0, 0.0, 1.0);
        let c = Vector3::zeros();

        // 8 faces (uses exactly 1 SIMD chunk)
        let mesh8 = create_faces(
            &[n, n, n, n, n, n, n, n],
            &[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            &[c, c, c, c, c, c, c, c],
        );
        let out8 = crate::physics::calculate_forces(&mesh8, &state, 1.225, sos, 1.8e-5);

        // 7 faces (uses exactly 7 scalar remainder, 0 SIMD)
        let mesh7 = create_faces(
            &[n, n, n, n, n, n, n],
            &[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            &[c, c, c, c, c, c, c],
        );
        let out7 = crate::physics::calculate_forces(&mesh7, &state, 1.225, sos, 1.8e-5);

        // 15 faces (1 SIMD chunk + 7 scalar remainder)
        let mesh15 = create_faces(
            &[n, n, n, n, n, n, n, n, n, n, n, n, n, n, n],
            &[
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
            ],
            &[c, c, c, c, c, c, c, c, c, c, c, c, c, c, c],
        );
        let out15 = crate::physics::calculate_forces(&mesh15, &state, 1.225, sos, 1.8e-5);

        // We know that exactly proportional elements should yield proportional additive forces.
        let force_per_face = out8.force / 8.0;
        assert_vec3_eq(out7.force, force_per_face * 7.0, 1e-2);
        assert_vec3_eq(out15.force, force_per_face * 15.0, 1e-2);
    }

    #[test]
    fn test_angular_velocity() {
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

        let out = crate::physics::calculate_forces(&mesh, &state, 1.225, sos, 1.8e-5);

        // A pure pitch acting on fins creates opposing drag loads that generate a pure damping torque on X
        // Mach = 10 / 340 = 0.0294
        // q = 0.5 * 1.225 * 10^2 = 61.25
        // Cp_windward = 2.0 / sqrt(1 - 0.0294^2) = 2.00086
        // Face 1 F_z = -122.55, causes x_torque = 1.0 * F_z = -122.55
        // Face 2 F_z = +122.55, causes x_torque = -1.0 * F_z = -122.55
        // Expected Torque X = -245.10
        assert_approx_eq(out.torque.x, -245.10, 1.0); // Damping torque opposite to rotation
        assert_vec3_eq(
            Vector3::new(0.0, out.torque.y, out.torque.z),
            Vector3::zeros(),
            1e-4,
        );
    }

    #[test]
    fn test_fast_inv_fifth_root() {
        // Range of typical Reynolds numbers for turbulent flow
        let test_vals: [f64; 6] = [1e4, 1e5, 1e6, 1e7, 1e8, 1e9];
        for &val in &test_vals {
            let exact = val.powf(-0.2);
            let approx = fast_inv_fifth_root(val);

            let rel_error = ((approx - exact) / exact).abs();

            // Verify that our fast approximation introduces less than 5% relative error
            assert!(
                rel_error < 0.05,
                "Approximation error too high for val: {}. Exact: {}, Approx: {}",
                val,
                exact,
                approx
            );
        }
    }

    #[test]
    fn test_turbulent_skin_friction_sea_level() {
        let sos = 340.0;
        let v_z = 200.0; // Fast enough for Re > 1e5
        let state = create_dummy_state(Vector3::new(0.0, 0.0, v_z), Vector3::zeros());

        // Sideways facing normals mean zero normal pressure component.
        let n = Vector3::new(1.0, 0.0, 0.0);
        let mesh = create_faces(&[n, n, n, n], &[1.0, 1.0, 1.0, 1.0], &[Vector3::zeros(); 4]);

        let out = crate::physics::calculate_forces(&mesh, &state, 1.225, sos, 1.8e-5);

        // n dot v_loc = 0 -> purely tangential sliding flow
        // Re = (1.225 * 200 * 1.0) / 1.8e-5 = 13,611,111.11 (>1e5)
        // Mach = 200 / 340 = 0.588
        // Knudsen = 1.482 * 0.588 / 13,611,111 = 6.4e-8 (negligible slip)
        // C_f = 0.074 * (Re)^-0.2 = ~0.00277
        // q = 0.5 * 1.225 * 200^2 = 24500
        // Expected tangent drag Fz = -24500 * 0.00277 * 4.0 = -271.44
        assert_approx_eq(out.force.z, -271.44, 2.0);

        // Base suction also applies because cos_theta <= 1e-6 (it is exactly 0).
        // Leeward Cp at Mach 0.588 is -0.1.
        // Expected normal force Fx = -(Cp) * q * Area = 0.1 * 24500 * 4.0 = 9800.0
        assert_approx_eq(out.force.x, 9800.0, 1.0);
        assert_approx_eq(out.force.y, 0.0, 1e-4);
    }

    #[test]
    fn test_laminar_skin_friction_low_reynolds() {
        let sos = 340.0;
        let v_z = 0.1; // Extremely slow, Re < 1e5
        let state = create_dummy_state(Vector3::new(0.0, 0.0, v_z), Vector3::zeros());

        let n = Vector3::new(1.0, 0.0, 0.0);
        let mesh = create_faces(&[n, n, n, n], &[1.0, 1.0, 1.0, 1.0], &[Vector3::zeros(); 4]);

        let out = crate::physics::calculate_forces(&mesh, &state, 1.225, sos, 1.8e-5);

        // Re = (1.225 * 0.1 * 1.0) / 1.8e-5 = 6805.5 (<1e5)
        // Laminar fallback C_f = 0.002
        // q = 0.5 * 1.225 * 0.1^2 = 0.006125
        // Expected tangent drag Fz = -0.006125 * 0.002 * 4.0 = -0.000049
        assert_approx_eq(out.force.z, -0.000049, 1e-6);

        // Base suction Cp = -0.1
        // Fx = 0.1 * 0.006125 * 4.0 = 0.00245
        assert_approx_eq(out.force.x, 0.00245, 1e-5);
    }

    #[test]
    fn test_subsonic_compressibility_scaling() {
        let sos = 340.0;
        let n = Vector3::new(0.0, 0.0, 1.0);
        let mesh = create_faces(&[n], &[1.0], &[Vector3::zeros()]);

        // Mach 0.1
        let v_z_01 = 0.1 * sos;
        let state_01 = create_dummy_state(Vector3::new(0.0, 0.0, v_z_01), Vector3::zeros());
        let out_01 = crate::physics::calculate_forces(&mesh, &state_01, 1.225, sos, 1.8e-5);
        let q_01 = 0.5 * 1.225 * v_z_01.powi(2);
        let cp_01 = out_01.force.z.abs() / q_01; // C_p = Force / (q * Area)
        assert_approx_eq(cp_01, 2.010, 0.01);

        // Mach 0.7
        let v_z_07 = 0.7 * sos;
        let state_07 = create_dummy_state(Vector3::new(0.0, 0.0, v_z_07), Vector3::zeros());
        let out_07 = crate::physics::calculate_forces(&mesh, &state_07, 1.225, sos, 1.8e-5);
        let q_07 = 0.5 * 1.225 * v_z_07.powi(2);
        let cp_07 = out_07.force.z.abs() / q_07;
        assert_approx_eq(cp_07, 2.800, 0.01);

        // Verify C_p increases as Mach -> 1.0 due to Prandtl-Glauert singularity
        assert!(cp_07 > cp_01);
    }
}
