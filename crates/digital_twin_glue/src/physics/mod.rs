use crate::prelude::*;
use nalgebra::Vector3;
use wide::{CmpGe, CmpGt, CmpLt, f64x8};

pub struct SolverOutput {
    pub force: Vector3<f64>,
    pub torque: Vector3<f64>,
}

/// The layers of the Earth's atmosphere used in our simulation.
///
/// The atmosphere changes as you go higher up. Rather than a single complex formula,
/// the International Standard Atmosphere (ISA) model breaks the sky into "layers".
/// Inside each layer, the temperature changes at a constant rate, which makes it
/// much easier to calculate the air pressure and density exactly.
///
/// The values here represent:
/// 1. Base Altitude (meters) - where the layer starts.
/// 2. Base Temperature (Kelvin) - the temperature at the bottom of the layer.
/// 3. Lapse Rate (Kelvin/meter) - how much the temperature changes for every meter you go up.
/// 4. Base Pressure (Pascals) - the air pressure at the bottom of the layer.
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

/// Figures out how thick the air is, how fast sound travels through it, and how sticky the air feels
/// at a certain altitude above the earth.
///
/// This uses the International Standard Atmosphere (ISA) to act like a table lookup.
/// It figures out which layer of the atmosphere the rocket is currently flying through
/// and then computes the specific air properties at that exact height.
///
/// Why do we need this?
/// - If the air is very dense (close to the ground) the rocket has more drag and more steering force.
/// - Knowing the speed of sound lets us figure out our "Mach" speed, which completely changes how air flows.
/// - Dynamic viscosity tells us how thick friction acts on the rocket's skin.
///
/// Returns a tuple: `(density [kg/m^3], speed_of_sound [m/s], dynamic_viscosity [kg/(m*s)])`.
pub fn lookup_atmosphere(altitude: f64) -> (f64, f64, f64) {
    if altitude > 100_000.0 {
        // High altitude boundary overrides for near vacuum.
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

    let mu0 = 1.716e-5;
    let t0 = 273.15;
    let c = 110.4;
    let dynamic_viscosity = mu0 * ((t0 + c) / (t + c)) * (t / t0).powf(1.5);

    (density, speed_of_sound, dynamic_viscosity)
}

/// A neatly packed bundle of everything about the air and the rocket's current movement.
///
/// Instead of constantly re-calculating or keeping track of the speed of sound,
/// velocity, and where the center of mass is, we gather all the "current status"
/// inputs into this struct. It makes passing this info around much easier as the
/// simulation loops through individual pieces of the rocket surface.
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

/// The core physics engine that estimates how the wind pushes against the rocket.
///
/// It doesn't treat the rocket as a single point, but as thousands of tiny flat
/// triangles (called a "mesh"). By understanding how air sweeps past each triangle,
/// it predicts where the rocket spins, slows down, or gets pushed off-course.
///
/// The math completely changes depending on how fast the rocket is going:
/// 1. **Subsonic (Under Mach 0.8):** Air acts sort of like water spilling around an object.
/// 2. **Transonic (Mach 0.8 to 1.2):** Air starts squeezing into shockwaves; math gets messy and unpredictable.
/// 3. **Supersonic (Mach 1.2 to 5.0):** Air hits hard in straight lines and expands loudly.
/// 4. **Hypersonic (Mach 5.0+):** The sheer force of hitting the air causes extreme heat, and the air molecules literally start breaking apart.
///
/// Returns a bundle (`SolverOutput`) with the net `force` (pushes) and net `torque` (twists).
pub fn calculate_forces(
    mesh: &Mesh,
    state: &MissileState,
    air_density: f64,
    speed_of_sound: f64,
    dyn_viscosity: f64,
) -> SolverOutput {
    let cm_local = Vector3::zeros();

    let v_inf = state.body_velocity.map(|v| v.value);
    let w = state.angular_velocity.map(|w| w.value);

    let v_inf_mag = v_inf.norm();
    let freestream_mach = v_inf_mag / speed_of_sound;

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

    if freestream_mach < 0.8 {
        // --- Region 1: Subsonic Flight ---
        //
        // Things are relatively calm here. The air smoothly splits
        // ahead of the rocket and slips around it. The rocket acts
        // almost like it's swimming in water.
        //
        // We use the Prandtl-Glauert rule here, which basically says:
        // As you get closer to the speed of sound, the air starts compressing
        // (squishing) more and more right at the nose of the rocket.
        integrate_mesh(
            mesh,
            &env,
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
        // --- Region 2: Transonic Flight ---
        //
        // This is a chaotic zone as the rocket breaks the sound barrier.
        // Some spots of the rocket move air faster than sound,
        // while other spots are moving the air slower. It's violent and
        // creates shock waves.
        //
        // Because of the chaos, there's no "perfect simple math" here.
        // Instead, we just guess it's somewhere between Subsonic behavior
        // and Supersonic behavior by creating a smooth sliding blend between the two.
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
        // --- Region 3: Supersonic Flight ---
        //
        // Your rocket is flying totally faster than the speed of sound.
        // By this point, the air can't get out of the way before the rocket hits it.
        // It hits hard, gets squeezed rapidly into "Shock waves", and then
        // quickly expands off the sides.
        //
        // We use a math rule created by someone named Ackeret.
        // It basically calculates angles of how sharply the air hits
        // each triangle and computes how much drag/push results.
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
        // --- Region 4: Hypersonic Flight ---
        //
        // Your rocket isn't flying anymore; it's smashing through the atmosphere
        // so wildly hard and fast that Newtonian Impact limits take over.
        // The air doesn't just hit the rocket, it hits it like solid pellets
        // bounding off.
        //
        // As you go faster than Mach 8, the energy gets so insanely hot
        // that the nitrogen and oxygen molecules literally break apart.
        // It's called "Molecular Dissociation", and it completely changes
        // the physics rules for drag and heat.
        integrate_mesh(
            mesh,
            &env,
            |cos_theta, local_mach| {
                let upper = local_mach.min(f64x8::splat(7.0));
                let blend =
                    ((upper - f64x8::splat(5.0)) / f64x8::splat(2.0)).max(f64x8::splat(0.0));

                let beta_5 = f64x8::splat((25.0_f64 - 1.0).sqrt());

                // Dissociation boundary forces Cp curve modifications above Mach 8 where Gamma=1.4 idealized models break.
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

/// A highly tuned piece of math to quickly calculate exactly `(x ^ -0.2)`.
///
/// In aerospace physics, turbulent air friction causes a ton of complex math.
/// Part of that formula involves calculating the fifth-root of a number rapidly
/// for thousands of triangles on our rocket.
///
/// If we do normal math, the CPU spends way too much time thinking.
/// Instead, we take the raw memory bytes of the decimal number, chop it in half,
/// do a little magic guessing, and fix it back up into an exact answer.
/// It's a famous old video game trick (like Quake's inverse square root), but
/// modified slightly for aerospace friction!
#[inline(always)]
fn fast_inv_fifth_root(x: f64) -> f64 {
    let i = x.to_bits();
    let j = 0x4cb8c612284ba400_u64 - i / 5;
    let y = f64::from_bits(j);
    let y2 = y * y;
    let y4 = y2 * y2;
    let y5 = y4 * y;

    // First Newton-Raphson approximation iteration.
    let y_next = y * (1.2 - 0.2 * x * y5);

    // Second iteration for increased precision.
    let y_next2 = y_next * y_next;
    let y_next4 = y_next2 * y_next2;
    let y_next5 = y_next4 * y_next;

    y_next * (1.2 - 0.2 * x * y_next5)
}

/// Calculate the length (or 'magnitude') of a 3D distance or speed simply using the Pythagorean theorem,
/// but specifically organized for our fast 'SIMD' processing batches (working on 8 segments at once).
#[inline(always)]
fn norm3(x: f64x8, y: f64x8, z: f64x8) -> f64x8 {
    ((x * x) + (y * y) + (z * z)).sqrt()
}

/// Calculates what physicists call a "Cross Product".
///
/// Simply put: if I have a 3D object rotating, and air hitting the object sideways,
/// what direction does the surface feel like it's dragging in?
/// This does that calculation, again packed perfectly into batches of 8 for performance.
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

/// A short helper function that grabs exactly 8 numbers out of a big list
/// safely so the CPU can pack them tightly for fast math loops.
#[inline(always)]
fn load_chunk(arr: &[f64], offset: usize) -> f64x8 {
    f64x8::new(arr[offset..offset + 8].try_into().unwrap())
}

/// An ugly hack we sometimes have to use.
///
/// Our code flies fastest when working on 8 pieces of data simultaneously `f64x8`.
/// But sometimes the rules of physics for one particular property haven't been re-written
/// into 8-pack batches yet. This helper splits the batch apart, runs the regular math
/// one at a time, and packs them back together as an 8-pack array.
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

/// What happens on the rear side of the rocket (the "leeward" side) where the wind isn't hitting directly?
///
/// Instead of a massive push of air into the rocket (positive pressure),
/// a small vacuum gap opens up behind the rocket (negative suction).
///
/// Because air gets lazier the faster the rocket flies past Mach 1.2, this number
/// tends to shrink back down predictably instead of skyrocketing infinitely.
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

/// The main workhorse function that calculates forces for 8 triangles at the exactly same time.
///
/// To make our simulator incredibly fast, we use a processor trick called SIMD.
/// Instead of doing math for triangle A, then triangle B, then triangle C...
/// SIMD lets us load 8 triangles into the CPU, and when we say "multiply by the wind speed",
/// the CPU multiplies all 8 at once in a single tick.
///
/// This function loops over the rocket, grabbing chunks of 8 triangles, finding out exactly
/// what angle they face the wind, calculating the pressure and skin friction, and then
/// summing everything up to find the total push and twist on the rocket snippet.
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

        // Load the face areas for these 8 elements into a standard vector block safely mapping boundary constraints.
        let area = load_chunk(&mesh.faces.area, offset);

        // Terminate cleanly for degenerate elements preventing divide faults propagating arbitrarily.
        let area_mask = area.simd_ge(f64x8::splat(1e-8));
        if area_mask.to_array() == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] {
            continue;
        }

        // Establish the moment arm radius vector directly to centroid mapped around object CM.
        let cx = load_chunk(&mesh.faces.centroid_x, offset);
        let cy = load_chunk(&mesh.faces.centroid_y, offset);
        let cz = load_chunk(&mesh.faces.centroid_z, offset);

        let rx = cx - cm_x;
        let ry = cy - cm_y;
        let rz = cz - cm_z;

        // Overlay rotational velocities directly to absolute translational paths computing combined flow.
        let (cross_x, cross_y, cross_z) = cross3(w_x, w_y, w_z, rx, ry, rz);
        let v_local_x = v_inf_x + cross_x;
        let v_local_y = v_inf_y + cross_y;
        let v_local_z = v_inf_z + cross_z;

        let v_mag = norm3(v_local_x, v_local_y, v_local_z);

        // Check local velocities bounding static stall limits strictly avoiding math limits aggressively.
        let v_mag_mask = v_mag.simd_ge(f64x8::splat(1e-3));
        let valid_mask = area_mask & v_mag_mask;
        if valid_mask.to_array() == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] {
            continue;
        }

        // Evaluate localized mach number bounded strictly by minimum velocity thresholds avoiding zero logic.
        let safe_v_mag = v_mag.max(f64x8::splat(1e-3));
        let local_mach = safe_v_mag / speed_of_sound_splat;

        // Determine specific incident vectors uniformly mapping inverted normalized flow directions efficiently.
        let v_dir_x = (f64x8::splat(0.0) - v_local_x) / safe_v_mag;
        let v_dir_y = (f64x8::splat(0.0) - v_local_y) / safe_v_mag;
        let v_dir_z = (f64x8::splat(0.0) - v_local_z) / safe_v_mag;

        let nx = load_chunk(&mesh.faces.normal_x, offset);
        let ny = load_chunk(&mesh.faces.normal_y, offset);
        let nz = load_chunk(&mesh.faces.normal_z, offset);

        // Derive dot product checking specific relative angle relationships (Negative indicates normal is facing flow).
        let cos_theta = (nx * v_dir_x) + (ny * v_dir_y) + (nz * v_dir_z);

        // Mask elements determining windward facing elements needing dynamic compressibility pressure distributions.
        let windward_mask = cos_theta.simd_lt(f64x8::splat(0.0));

        // Evaluate generalized aerodynamic compression coefficient across localized faces cleanly matching current regime explicitly.
        let cp_windward = calc_windward_cp(cos_theta.abs(), local_mach);
        let c_p = windward_mask.blend(cp_windward, leeward_cp_splat);

        // Utilize freestream Reynolds limits directly mapping flow energy correctly applying specific transition dynamics.
        let reynolds = (air_density_splat * safe_v_mag * length_ref_splat) / dyn_viscosity_splat;
        let turbulent_mask = reynolds.simd_gt(f64x8::splat(1e5));

        // Establish localized Knudsen number accurately applying specific slip friction bounds to high attitude molecular drag.
        let safe_reynolds = reynolds.max(f64x8::splat(1e-5));
        let knudsen = f64x8::splat(1.482) * local_mach / safe_reynolds;
        let slip_correction = f64x8::splat(1.0) / (f64x8::splat(1.0) + knudsen);

        // Approximate skin friction explicitly replacing standard power branches aggressively reducing 0.2 exponential overhead.
        let cp_turb = map_f64x8(safe_reynolds, |r| 0.074 * fast_inv_fifth_root(r));

        // Mask specific transitioned boundary limits reliably blending final resulting limits exactly mapping specific friction models.
        let c_f = turbulent_mask.blend(cp_turb, f64x8::splat(0.002)) * slip_correction;

        // Establish raw dynamic pressure values acting cleanly universally against every geometry component mathematically.
        let q = f64x8::splat(0.5) * air_density_splat * safe_v_mag * safe_v_mag;

        // Extract localized uniform structural absolute resulting bounds reliably distributing net forces symmetrically.
        let fn_mag = q * c_p * area;
        let fn_x = (f64x8::splat(0.0) - nx) * fn_mag;
        let fn_y = (f64x8::splat(0.0) - ny) * fn_mag;
        let fn_z = (f64x8::splat(0.0) - nz) * fn_mag;

        // Determine generalized surface tangential trajectories mapping flow precisely mapping lateral directions cleanly.
        let vt_x = v_dir_x - (nx * cos_theta);
        let vt_y = v_dir_y - (ny * cos_theta);
        let vt_z = v_dir_z - (nz * cos_theta);
        let vt_mag = norm3(vt_x, vt_y, vt_z);

        // Sanitize tangential limits ignoring directly perpendicular incidence mapping cleanly.
        let tangent_mask = vt_mag.simd_gt(f64x8::splat(1e-6));
        let safe_vt_mag = vt_mag.max(f64x8::splat(1e-6));
        let tangent_dir_x = vt_x / safe_vt_mag;
        let tangent_dir_y = vt_y / safe_vt_mag;
        let tangent_dir_z = vt_z / safe_vt_mag;

        let ft_mag = q * c_f * area;

        // Apply skin-friction explicitly parallel across specific valid boundary masks limiting arbitrary faults.
        let ft_x = tangent_mask.blend(tangent_dir_x * ft_mag, f64x8::splat(0.0));
        let ft_y = tangent_mask.blend(tangent_dir_y * ft_mag, f64x8::splat(0.0));
        let ft_z = tangent_mask.blend(tangent_dir_z * ft_mag, f64x8::splat(0.0));

        // Incorporate specific normal limits and valid masks limiting specific valid total sum structures globally.
        let f_x = valid_mask.blend(fn_x + ft_x, f64x8::splat(0.0));
        let f_y = valid_mask.blend(fn_y + ft_y, f64x8::splat(0.0));
        let f_z = valid_mask.blend(fn_z + ft_z, f64x8::splat(0.0));

        // Consolidate total applied boundary force sums accurately preserving state vectors.
        force_x += f_x;
        force_y += f_y;
        force_z += f_z;

        // Resolve moment components mapping absolute generated bounds seamlessly matching object references strictly.
        let (t_x, t_y, t_z) = cross3(rx, ry, rz, f_x, f_y, f_z);
        torque_x += t_x;
        torque_y += t_y;
        torque_z += t_z;
    }

    // Reduces the integrated raw wide registers down toward fixed Cartesian floats sequentially mapping the final force vectors efficiently.
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

/// The cleanup crew for the triangles that got left over.
///
/// Because our super-fast SIMD function above only works in chunks of 8 triangles,
/// what happens if the rocket has 10 triangles total? The first 8 triangles run fast,
/// but there are 2 leftovers at the end.
///
/// This function is standard "scalar" math it simply calculates the leftover triangles
/// one by one. By only doing this for the last 0 to 7 triangles, it's fast enough
/// and prevents the program from crashing by trying to load empty triangles.
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

        // Terminate specific minimal elements safely preventing divide by zero states mapping strictly.
        if area_scalar < 1e-8 {
            continue;
        }

        // Map relative torque arms mapping directly against primary center of mass coordinates securely.
        let r_scalar = Vector3::new(
            mesh.faces.centroid_x[i],
            mesh.faces.centroid_y[i],
            mesh.faces.centroid_z[i],
        ) - env.cm;

        // Isolate primary surface normals acting orthogonally against bounds predictably.
        let n_scalar = Vector3::new(
            mesh.faces.normal_x[i],
            mesh.faces.normal_y[i],
            mesh.faces.normal_z[i],
        );

        // Derive specific localized total absolute trajectory components dynamically bounded cleanly.
        let loc_v = env.v_inf + env.w.cross(&r_scalar);
        let vm = loc_v.norm();
        if vm < 1e-3 {
            continue;
        }

        let lmach = vm / env.speed_of_sound;

        // Limit incidence angle checking negative absolute boundaries defining orientation completely safely.
        let d = -loc_v / vm;
        let cdt = n_scalar.dot(&d);

        // Assign windward bounds explicitly substituting specific regimes using isolated fallback rules cleanly.
        let cp_w = calc_windward_cp_scalar(cdt.abs(), lmach);
        let cp = if cdt < 0.0 { cp_w } else { leeward_cp };

        // Process boundary slip factors smoothing high turbulence specifically addressing supersonic and hypersonics accurately.
        let rey = (env.air_density * vm * env.length_ref) / env.dyn_viscosity;
        let knudsen = 1.482 * lmach / rey.max(1e-5);
        let slip_correction = 1.0 / (1.0 + knudsen);

        let cf = if rey > 1e5 {
            0.074 * fast_inv_fifth_root(rey)
        } else {
            0.002
        } * slip_correction;

        // Incorporate final dynamic constants limiting absolute pressure values matching specific densities natively.
        let qq = 0.5 * env.air_density * vm * vm;

        // Apply raw computed pressure coefficients projecting perfectly mapped loads over isolated element areas explicitly.
        let fnrm = -n_scalar * (qq * cp * area_scalar);
        let vtan = d - n_scalar * cdt;
        let vtm = vtan.norm();
        let ftan = if vtm > 1e-6 {
            (vtan / vtm) * (qq * cf * area_scalar)
        } else {
            Vector3::zeros()
        };

        // Complete final isolated net moment calculation projecting fully assembled element torque loads cleanly.
        let f_el = fnrm + ftan;
        total_force += f_el;
        total_torque += r_scalar.cross(&f_el);
    }

    (total_force, total_torque)
}

/// The master controller for adding up all the aerodynamic forces.
///
/// It manages the work by splitting the mesh into completely full 8-triangle chunks,
/// passing those to our lightning-fast SIMD processor (`integrate_simd_chunks`),
/// and then passing any leftover parts to the slow-but-steady scalar backup
/// (`integrate_scalar_remainder`).
///
/// Finally, it takes the push forces from both of those groups and lumps them into
/// a single total force and total twist representing the entire rocket shape.
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
    // Derive absolute leeward boundaries bounding uniformly parallel static Mach constraints logically.
    let leeward_cp = get_leeward_cp(env.freestream_mach);

    // Divvy absolute bounds correctly generating chunks bounded tightly against `8` triangle groups optimally.
    let n_faces = mesh.faces.area.len();
    let chunks = n_faces / 8;
    let remainder = n_faces % 8;

    // Apply primary bulk integration loop returning accurately formed moment limits cleanly avoiding condition blocks completely.
    let (simd_force, simd_torque) =
        integrate_simd_chunks(mesh, env, leeward_cp, chunks, &calc_windward_cp);

    // Apply trailing remainder operations cleanly matching completely exact logical steps natively cleanly.
    let rem_start = n_faces - remainder;
    let (rem_force, rem_torque) = integrate_scalar_remainder(
        mesh,
        env,
        leeward_cp,
        rem_start,
        n_faces,
        &calc_windward_cp_scalar,
    );

    // Consolidate complete aggregate body outputs strictly mapping resolved state vectors accurately natively.
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
        let v_z = 0.5 * sos;
        let state = create_dummy_state(Vector3::new(0.0, 0.0, v_z), Vector3::zeros());

        let n = Vector3::new(0.0, 0.0, 1.0);
        let mesh = create_faces(&[n, n, n, n], &[1.0, 1.0, 1.0, 1.0], &[Vector3::zeros(); 4]);

        let out = crate::physics::calculate_forces(&mesh, &state, 1.225, sos, 1.8e-5);

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
        let v_z = 1.0 * sos;
        let state = create_dummy_state(Vector3::new(0.0, 0.0, v_z), Vector3::zeros());

        let n = Vector3::new(0.0, 0.0, 1.0);
        let mesh = create_faces(&[n, n, n, n], &[1.0, 1.0, 1.0, 1.0], &[Vector3::zeros(); 4]);

        let out = crate::physics::calculate_forces(&mesh, &state, 1.225, sos, 1.8e-5);

        assert_approx_eq(out.force.z, -899003.54, 2.0);
    }

    #[test]
    fn test_lateral_aerodynamics_angle_of_attack() {
        let sos = 340.0;

        let state = create_dummy_state(Vector3::new(100.0, 0.0, 100.0), Vector3::zeros());
        let n = Vector3::new(0.0, 0.0, 1.0);
        let mesh = create_faces(&[n], &[1.0], &[Vector3::zeros()]);

        let out = crate::physics::calculate_forces(&mesh, &state, 1.225, sos, 1.8e-5);

        assert_approx_eq(out.force.z, -19050.26, 1.0);

        assert!(out.force.x < -0.0);
        assert_approx_eq(out.force.y, 0.0, 1e-6);
    }

    #[test]
    fn test_supersonic_force() {
        let sos = 340.0;
        let v_z = 3.0 * sos;
        let state = create_dummy_state(Vector3::new(0.0, 0.0, v_z), Vector3::zeros());

        let n = Vector3::new(0.0, 0.0, 1.0);
        let mesh = create_faces(&[n, n, n, n], &[1.0, 1.0, 1.0, 1.0], &[Vector3::zeros(); 4]);

        let out = crate::physics::calculate_forces(&mesh, &state, 1.225, sos, 1.8e-5);

        assert_approx_eq(out.force.z, -1802401.04, 2.0);
    }

    #[test]
    fn test_hypersonic_force() {
        let sos = 340.0;
        let v_z = 6.0 * sos;
        let state = create_dummy_state(Vector3::new(0.0, 0.0, v_z), Vector3::zeros());

        let n = Vector3::new(0.0, 0.0, 1.0);
        let mesh = create_faces(&[n, n, n, n], &[1.0, 1.0, 1.0, 1.0], &[Vector3::zeros(); 4]);

        let out = crate::physics::calculate_forces(&mesh, &state, 1.225, sos, 1.8e-5);

        assert_approx_eq(out.force.z, -11456381.89, 2.0);
    }

    #[test]
    fn test_degenerate_faces_ignored() {
        let sos = 340.0;
        let state = create_dummy_state(Vector3::new(0.0, 0.0, sos), Vector3::zeros());

        let n = Vector3::new(0.0, 0.0, 1.0);
        let mesh = create_faces(
            &[n, n, n, n],
            &[1.0, 1e-10, 1e-10, 1e-10],
            &[Vector3::zeros(); 4],
        );
        let out_degenerate = crate::physics::calculate_forces(&mesh, &state, 1.225, sos, 1.8e-5);

        let mesh2 = create_faces(&[n], &[1.0], &[Vector3::zeros()]);
        let out_single = crate::physics::calculate_forces(&mesh2, &state, 1.225, sos, 1.8e-5);

        assert_vec3_eq(out_degenerate.force, out_single.force, 1e-5);
    }

    #[test]
    fn test_stagnant_bailout() {
        let sos = 340.0;

        let state = create_dummy_state(Vector3::new(0.0, 0.0, 1e-4), Vector3::zeros());
        let n = Vector3::new(0.0, 0.0, 1.0);
        let mesh = create_faces(&[n, n, n, n], &[1.0, 1.0, 1.0, 1.0], &[Vector3::zeros(); 4]);

        let out = crate::physics::calculate_forces(&mesh, &state, 1.225, sos, 1.8e-5);

        assert_vec3_eq(out.force, Vector3::zeros(), 1e-10);
    }

    #[test]
    fn test_simd_vs_scalar_remainder() {
        let sos = 340.0;
        let state = create_dummy_state(Vector3::new(0.0, 0.0, sos * 2.0), Vector3::zeros());

        let n = Vector3::new(0.0, 0.0, 1.0);
        let c = Vector3::zeros();

        let mesh8 = create_faces(
            &[n, n, n, n, n, n, n, n],
            &[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            &[c, c, c, c, c, c, c, c],
        );
        let out8 = crate::physics::calculate_forces(&mesh8, &state, 1.225, sos, 1.8e-5);

        let mesh7 = create_faces(
            &[n, n, n, n, n, n, n],
            &[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            &[c, c, c, c, c, c, c],
        );
        let out7 = crate::physics::calculate_forces(&mesh7, &state, 1.225, sos, 1.8e-5);

        let mesh15 = create_faces(
            &[n, n, n, n, n, n, n, n, n, n, n, n, n, n, n],
            &[
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
            ],
            &[c, c, c, c, c, c, c, c, c, c, c, c, c, c, c],
        );
        let out15 = crate::physics::calculate_forces(&mesh15, &state, 1.225, sos, 1.8e-5);

        let force_per_face = out8.force / 8.0;
        assert_vec3_eq(out7.force, force_per_face * 7.0, 1e-2);
        assert_vec3_eq(out15.force, force_per_face * 15.0, 1e-2);
    }

    #[test]
    fn test_angular_velocity() {
        let sos = 340.0;

        let w = Vector3::new(10.0, 0.0, 0.0);
        let state = create_dummy_state(Vector3::zeros(), w);

        let mut mesh = Mesh::default();
        mesh.faces.normal_x.push(0.0);
        mesh.faces.normal_y.push(0.0);
        mesh.faces.normal_z.push(1.0);
        mesh.faces.area.push(1.0);
        mesh.faces.centroid_x.push(0.0);
        mesh.faces.centroid_y.push(1.0);
        mesh.faces.centroid_z.push(0.0);

        mesh.faces.normal_x.push(0.0);
        mesh.faces.normal_y.push(0.0);
        mesh.faces.normal_z.push(-1.0);
        mesh.faces.area.push(1.0);
        mesh.faces.centroid_x.push(0.0);
        mesh.faces.centroid_y.push(-1.0);
        mesh.faces.centroid_z.push(0.0);

        let out = crate::physics::calculate_forces(&mesh, &state, 1.225, sos, 1.8e-5);

        assert_approx_eq(out.torque.x, -245.10, 1.0);
        assert_vec3_eq(
            Vector3::new(0.0, out.torque.y, out.torque.z),
            Vector3::zeros(),
            1e-4,
        );
    }

    #[test]
    fn test_fast_inv_fifth_root() {
        let test_vals: [f64; 6] = [1e4, 1e5, 1e6, 1e7, 1e8, 1e9];
        for &val in &test_vals {
            let exact = val.powf(-0.2);
            let approx = fast_inv_fifth_root(val);

            let rel_error = ((approx - exact) / exact).abs();

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
        let v_z = 200.0;
        let state = create_dummy_state(Vector3::new(0.0, 0.0, v_z), Vector3::zeros());

        let n = Vector3::new(1.0, 0.0, 0.0);
        let mesh = create_faces(&[n, n, n, n], &[1.0, 1.0, 1.0, 1.0], &[Vector3::zeros(); 4]);

        let out = crate::physics::calculate_forces(&mesh, &state, 1.225, sos, 1.8e-5);

        assert_approx_eq(out.force.z, -271.44, 2.0);

        assert_approx_eq(out.force.x, 9800.0, 1.0);
        assert_approx_eq(out.force.y, 0.0, 1e-4);
    }

    #[test]
    fn test_laminar_skin_friction_low_reynolds() {
        let sos = 340.0;
        let v_z = 0.1;
        let state = create_dummy_state(Vector3::new(0.0, 0.0, v_z), Vector3::zeros());

        let n = Vector3::new(1.0, 0.0, 0.0);
        let mesh = create_faces(&[n, n, n, n], &[1.0, 1.0, 1.0, 1.0], &[Vector3::zeros(); 4]);

        let out = crate::physics::calculate_forces(&mesh, &state, 1.225, sos, 1.8e-5);

        assert_approx_eq(out.force.z, -0.000049, 1e-6);

        assert_approx_eq(out.force.x, 0.00245, 1e-5);
    }

    #[test]
    fn test_subsonic_compressibility_scaling() {
        let sos = 340.0;
        let n = Vector3::new(0.0, 0.0, 1.0);
        let mesh = create_faces(&[n], &[1.0], &[Vector3::zeros()]);

        let v_z_01 = 0.1 * sos;
        let state_01 = create_dummy_state(Vector3::new(0.0, 0.0, v_z_01), Vector3::zeros());
        let out_01 = crate::physics::calculate_forces(&mesh, &state_01, 1.225, sos, 1.8e-5);
        let q_01 = 0.5 * 1.225 * v_z_01.powi(2);
        let cp_01 = out_01.force.z.abs() / q_01;
        assert_approx_eq(cp_01, 2.010, 0.01);

        let v_z_07 = 0.7 * sos;
        let state_07 = create_dummy_state(Vector3::new(0.0, 0.0, v_z_07), Vector3::zeros());
        let out_07 = crate::physics::calculate_forces(&mesh, &state_07, 1.225, sos, 1.8e-5);
        let q_07 = 0.5 * 1.225 * v_z_07.powi(2);
        let cp_07 = out_07.force.z.abs() / q_07;
        assert_approx_eq(cp_07, 2.800, 0.01);

        assert!(cp_07 > cp_01);
    }
}
