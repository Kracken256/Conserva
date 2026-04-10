use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};
use uom::si::f64::{Angle, AngularVelocity, Force, Length, Mass, Time, Velocity};
use uom::si::time::second;

/// Defines the overarching geometric archetype of the missile's forwardmost aerodynamic surface.
/// The nosecone plays a disproportionately massive role in dictating the vehicle's total
/// supersonic wave drag and sub-sonic pressure distribution. Selecting the proper shape
/// requires balancing internal volume requirements against external thermal heating and drag
/// penalties across different Mach regimes.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "shape", rename_all = "snake_case")]
pub enum NoseconeShape {
    /// A standard conical nosecone. This shape features straight lines extending directly from the
    /// tip to the main body cylinder. It is simple to manufacture and provides decent supersonic
    /// drag characteristics. However, it may not be the most aerodynamically efficient shape for
    /// all flight regimes, exhibiting separated flow at the shoulder.
    Conical {
        /// The axial length of the conical section.
        length: Length,

        /// Radius of spherical blunting at the tip. None for infinitely sharp.
        blunting_radius: Option<Length>,
    },

    /// An ogive nosecone shape. This profile curves smoothly into the body, offering a better
    /// aerodynamic transition than a basic cone. It is commonly used to reduce wave drag in
    /// supersonic flight while maintaining a sharp tip. It requires more complex manufacturing
    /// but yields significantly improved aerodynamic performance and internal payload volume.
    Ogive {
        /// The axial length of the ogive section.
        length: Length,

        /// Radius of spherical blunting at the tip.
        blunting_radius: Option<Length>,

        /// If specified, generates a Secant Ogive with this profile curve radius. If None,
        /// computes a Tangent Ogive.
        secant_radius: Option<Length>,
    },

    /// An elliptical nosecone design. This geometry utilizes a classic ellipse curve to provide
    /// a blunt, high-volume forward section. Blunt profiles like this are frequently preferred in
    /// subsonic rockets or extreme hypersonic regimes where thermal heating outpaces wave drag
    /// concerns. It provides maximum internal payload space at the extreme forward tip.
    Elliptical {
        /// The axial length of the elliptical section.
        length: Length,
    },

    /// A parabolic nosecone configuration. This geometry features a blunter tip compared to the
    /// ogive shape while still curving smoothly into the main body cylinder. It is often utilized
    /// to manage aerodynamic heating at very high speeds, as the blunter tip dissipates thermal
    /// energy more effectively. The shape factor can be varied from a cone to a full parabola.
    Parabolic {
        /// The axial length of the parabolic section.
        length: Length,

        /// Shape factor K. 0.0 is a cone, 1.0 is a full parabola.
        k_factor: f64,

        /// Radius of spherical blunting at the tip.
        blunting_radius: Option<Length>,
    },

    /// A power-series geometric profile. This advanced contour relies on a mathematical power
    /// factor to define the sweeping shape curve from the tip to the shoulder. It enables
    /// meticulous optimization of the aerodynamic profile according to specialized flow
    /// conditions or custom constraints. Fractional powers typically create blunt tips while
    /// larger powers yield sharp, elongated needle profiles.
    PowerSeries {
        /// The axial length of the power-series section.
        length: Length,

        /// Power factor n. Shape curve: y = R * (x/L)^n
        n: f64,

        /// Radius of spherical blunting at the tip.
        blunting_radius: Option<Length>,
    },

    /// A Haack series aerodynamic contour. Discovered mathematically to minimize wave drag, these
    /// shapes are not constructed from geometric primitives but rather from analytical drag equations.
    /// Variations include the Von Karman (LD-Haack) and LV-Haack profiles, tailored for lowest drag
    /// given a defined length/diameter or volume/diameter ratio respectively. They are heavily
    /// utilized in professional high-altitude telemetry vehicles.
    Haack {
        /// The axial length of the Haack series section.
        length: Length,

        /// C multiplier for Haack series. 0.0 for LD-Haack (Von Karman), 1.0/3.0 for LV-Haack.
        c_factor: f64,

        /// Radius of spherical blunting at the tip.
        blunting_radius: Option<Length>,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "profile", rename_all = "snake_case")]
pub enum FinEdgeProfile {
    Straight,

    /// An arc extending away from the straight chord line by the specified `depth`. A
    /// positive depth value bows the edge outward, creating a convex profile. A
    /// negative depth value scoops the edge inward, creating a concave profile. This
    /// parameter enables sophisticated aerodynamic contouring.
    Curved {
        depth: Length,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FinGeometry {
    /// The total number of fins symmetrically distributed around the missile body. This
    /// directly impacts the aerodynamic stability and total surface area of the
    /// vehicle. Typical values range from 3 to 4 depending on the desired control
    /// authority. It is assumed the fins are evenly spaced radially.
    pub num_fins: u32,

    /// The axial distance measured from the absolute tip of the nose to the leading
    /// edge of the fins at their root attachment point. This value positions the fin
    /// assembly longitudinally along the body. Accurate placement here is critical for
    /// determining the center of pressure. It dictates the lever arm available for
    /// aerodynamic stabilization.
    pub offset_from_nose: Length,

    /// The physical length of the fin's root edge, defining the structural attachment
    /// footprint to the main body. This chord length provides the physical base for
    /// transferring aerodynamic loads into the airframe. A longer root chord generally
    /// increases the structural integrity of the fin joint. It spans from the root
    /// leading edge to the root trailing edge.
    pub root_chord: Length,

    /// The physical length of the fin's outermost tip edge, which runs parallel to the
    /// longitudinal axis of the body. If this value is set to zero, the fin geometry
    /// collapses into a simple triangular shape. A non-zero tip chord creates a
    /// trapezoidal or swept platform. It significantly influences the tip vortex and
    /// aerodynamic efficiency.
    pub tip_chord: Length,

    /// The radially outward distance extending from the fin's base root all the way to
    /// its outermost tip. This measurement defines the total spanwise reach of the
    /// aerodynamic surface into the free stream. A larger span increases the
    /// stabilizing moment and lift generation. It must be balanced against mechanical
    /// bending stresses and launch rail clearances.
    pub span: Length,

    /// The axial distance running parallel to the body from the root's leading edge to
    /// the tip's leading edge. This parameter defines the mechanical sweep of the fin
    /// structure. Positive sweep delays the onset of transonic drag rise and moves the
    /// aerodynamic center aft. It is a vital parameter for high-speed stability tuning.
    pub sweep_length: Length,

    /// The structural thickness of the fin extending laterally across its cross-
    /// section. This defines the physical volume and material heft of the control
    /// surface. Thicker fins provide greater mechanical strength but at the cost of
    /// increased parasitic drag. It is typically sized based on the material yield
    /// strength and flutter margins.
    pub thickness: Length,

    /// The geometrical contour logic designated for the front-facing leading edge of
    /// the fin. This allows the forward boundary to deviate from a standard straight
    /// line into complex curves. It dictates how the fluid flow initially impacts and
    /// splits around the fin structure. Customizing this profile can optimize sub-sonic
    /// and supersonic wave distributions.
    pub leading_edge_profile: FinEdgeProfile,

    /// The geometrical contour logic designated for the rear-facing trailing edge of
    /// the fin. This allows the aft boundary to be uniquely shaped independently of the
    /// leading edge. It determines the flow separation characteristics and base drag
    /// profile. Sweeping or curving this edge can minimize turbulent wake effects.
    pub trailing_edge_profile: FinEdgeProfile,

    /// The lateral distance over which the leading and trailing edges uniformly taper
    /// structurally to a sharp point. If this value is strictly 0.0, the edges remain
    /// perfectly flat and unchamfered. Adding a chamfer significantly reduces profile
    /// drag by eliminating blunt faces. It ensures a streamlined flow separation and
    /// reattachment regime.
    pub edge_chamfer: Length,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissileGeometryConfig {
    /// The physical shape type and parameters describing the nosecone contour and
    /// length. Different geometries (Ogive, Conical, Parabolic) possess distinct
    /// aerodynamic drag profiles.
    pub nosecone_shape: NoseconeShape,

    /// The total length of the main cylindrical body of the missile. It defines the primary
    /// aerodynamic acting surface and total volume. This measurement actively dictates the
    /// dimensional space available for the payload, electronics, and propellant.
    pub cylindrical_body_length: Length,

    /// The maximum diameter of the missile's main body tube. This measurement bounds the internal
    /// volume and dictates the frontal reference area utilized in aerodynamic drag estimations. It
    /// restricts component sizing throughout the chassis.
    pub diameter: Length,

    /// Detailed geometry parameterizing the precise aerodynamic and structural
    /// properties of the main fins.
    pub fin_set: FinGeometry,

    /// A time-series interpolation mapping that models the 3D center of gravity relative to the
    /// vehicle's geometric center over the duration of the flight. As solid motor propellant
    /// depletes, the true mass center dynamically shifts, forcing physics recalculations.
    /// Accurately tracking this curve ensures true-to-life rotational kinematic simulations.
    pub cg_curve: Vec<(Time, Vector3<Length>)>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissileControllerConfig {
    /// The proportional gain for the thrust vector control and fin pitch control loop. It dictates
    /// how aggressively the controller immediately applies correcting moments relative to the
    /// absolute current pitch error scale. High values quicken responsiveness but run the risk of
    /// triggering severe oscillations.
    pub pitch_pi_kp: Vec<(Time, f64)>,

    /// The integral gain tuning constant for the automated pitch control loop. It aggregates and
    /// counters accumulated error histories to wash out steady-state offsets—like persistent
    /// gravity sag. However, an excessively large value often manifests as delayed overshoots and
    /// integral windups.
    pub pitch_pi_ki: Vec<(Time, f64)>,

    /// The unscaled proportional gain factor controlling the horizontal thrust vector control and
    /// fin yaw loop. It produces immediate physical reactions when the current telemetry heading
    /// deviates from the navigational target heading. Proper tuning tightens flight tracking
    /// capabilities against active lateral constraints.
    pub yaw_pi_kp: Vec<(Time, f64)>,

    /// The foundational integral multiplier backing the horizontal yaw control loop tuning. It
    /// steadily ramps up correction commands if the heading persistently lingers off-band due to
    /// unresolved continuous disturbances. This function acts primarily to negate sustained
    /// environmental crosswinds over an extended trajectory.
    pub yaw_pi_ki: Vec<(Time, f64)>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissileEngineConfig {
    /// An interpolated time-series array establishing the engine's functional thrust mapping
    /// profile over its comprehensive burn span. Each vertex binds a timestamp to the
    /// maximum total force the exhaust plume exerts on the airframe. The completion of this
    /// trace fundamentally indicates motor burnout and aerodynamic coasting operations.
    pub motor_impulse_curve: Vec<(Time, Force)>,

    /// The maximum physical deflection angle allowed by the Thrust Vector Control (TVC) nozzle
    /// gimbal actuators. This hard constraint limits the magnitude of correcting moments the
    /// engine can generate in a single tick regardless of PI requests. Proper tuning ensures
    /// the guidance software respects physical hardware servo limits without commanding
    /// over-travel.
    pub max_tvc_angle: Angle,

    /// The absolute maximum rotational velocity at which the TVC nozzle actuators can physically
    /// pivot to a new commanded position. This slew rate introduces a critical real-world delay,
    /// meaning large commands take several milliseconds to execute. Modeling this constraint
    /// prevents the simulation from performing impossible instantaneous control surface snaps.
    pub tvc_slew_rate: AngularVelocity,

    /// The exponential time constant (tau) defining the first-order lag of the TVC actuator's
    /// response from an idle or steady state towards the newly commanded displacement.
    /// This latency bounds the electromechanical settling delay inherent to physical servo
    /// systems translating voltages into momentum. A non-zero value cushions abrupt PI step
    /// commands dynamically.
    pub tvc_activation_delay: Time,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissileMassConfig {
    /// The unladen, baseline total mass of the missile configuration completely devoid of
    /// consumable propellant. This rigid figure encapsulates the airframe structures, computing
    /// electronics, inert payload hardware, and empty motor casings. It establishes the physical
    /// absolute lower bounding limit for weight computations.
    pub dry_mass: Mass,

    /// The total accumulated gross mass of the missile on the pad, completely saturated with
    /// propellants and primed for initial flight sequences. It acts as the upper culmination value
    /// tying the inert payload to the completely loaded energy potential. It predominantly drives
    /// the scale of dynamic initial liftoff thrust-to-weight demands.
    pub wet_mass: Mass,

    /// An interconnected mapping series matching elapsed simulation times to the active physical,
    /// current structural mass of the air vehicle. Interpolation tracking throughout this curve
    /// accurately replicates physical matter reduction across the burn envelopes and distinct
    /// structural staging deployments. The resulting transient variable extensively modifies
    /// ongoing gravitational and rotational inertias bindings.
    pub mass_curve: Vec<(Time, Mass)>,

    /// An interpolated time-series mapping that tracks the dynamic 3x3 inertia tensor
    /// matrix over the duration of the motor burn. As solid propellant mass depletes, the
    /// vehicle's structural resistance to pitch, yaw, and roll torques constantly shifts.
    /// The physics engine continually evaluates this curve to accurately model changing
    /// rotational kinetics and moment distributions throughout the entire flight envelope.
    pub inertia_tensor_curve: Vec<(Time, Matrix3<f64>)>,
}

/// Delineates the atmospheric boundary conditions and weather parameters.
/// It configures the baseline ambient wind vectors operating across the planetary body.
/// Additionally, it parameterizes advanced turbulence profiles utilizing standardized models like Dryden formulas.
/// Collectively, these parameters rigidly dictate aerodynamic density adjustments and unsteady air behaviors.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EnvironmentalConfig {
    /// Baseline continuous three-dimensional flow vector dictating total wind behavior.
    /// It effectively acts as a persistent kinematic translation directly shifting observed global airspeed variables.
    /// This metric strictly enforces gross drift constraints over extended integration distances.
    pub wind_velocity: Vector3<Velocity>,

    /// A multiplicative scaling factor applied atop internal turbulent noise layers.
    /// Values above zero spawn chaotic high-frequency flutter reflecting violent, unresolved thermals tracking past.
    /// Setting this rigidly limits randomized flight path deviations strictly required for realistic tuning.
    pub turbulence_intensity: Velocity,
}

impl Default for EnvironmentalConfig {
    fn default() -> Self {
        use uom::si::velocity::meter_per_second;
        Self {
            wind_velocity: Vector3::new(
                Velocity::new::<meter_per_second>(0.0),
                Velocity::new::<meter_per_second>(0.0),
                Velocity::new::<meter_per_second>(0.0),
            ),
            turbulence_intensity: Velocity::new::<meter_per_second>(0.0),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissileConfig {
    /// Gathers all defining structural boundary measurements obligatory for
    /// constructing the three-dimensional airframe chassis. This envelope fundamentally
    /// incorporates base diameters alongside length offsets essential for assembling
    /// realistic geometry structures. Consequently, it dominates the resulting baseline
    /// estimations mapped during physics solver friction integration.
    pub geometry: MissileGeometryConfig,

    /// Orchestrates the internally verified tuning scale mappings for the main execution guidance
    /// loops regulating flight path handling. The values embedded here instruct precisely how raw
    /// orientation telemetry discrepancies resolve into executable fin-actioned deflections during
    /// tick iterations. Calibrating this unit accurately assures rigid trajectory maintenance
    /// without yielding active vehicle authorities to chaotic harmonic breakdowns.
    pub controller: MissileControllerConfig,

    /// Delineates the propulsive capacity constraints modeling the primary energy propulsion
    /// subsystem capabilities attached to the missile structure. It intrinsically establishes the
    /// maximum bounds for acceleratory thrust operations applied during physics force integrations
    /// through time-mapped active profile evaluations. Therefore, it injects the critical forward
    /// momentum indispensable for achieving dynamic flight stability.
    pub engine: MissileEngineConfig,

    /// Manages the fundamental kinematic state constraints, including the inert bounds
    /// spanning the peak physical wet deployment masses down through totally depleted dry casings.
    /// It dictates the structural moment distribution tracking across non-linear transient burn
    /// envelopes seamlessly synchronized within physics derivations. The rigid rotational matrix
    /// tensors bundled strictly enforce handling rigidity constraints applied against the physics
    /// engine torques.
    pub mass: MissileMassConfig,

    /// Maps the external environmental and meteorological constraints acting directly on the system.
    /// Parameters nested here specify dynamic properties capturing turbulent thermal profiles and prevailing wind drift.
    /// Thus, physical aerodynamic iterations synchronize completely with shifting realistic air properties.
    #[serde(default)]
    pub environment: EnvironmentalConfig,
}

impl NoseconeShape {
    /// Gets the structural length of the nosecone
    pub fn length(&self) -> Length {
        match self {
            Self::Conical { length, .. } => *length,
            Self::Ogive { length, .. } => *length,
            Self::Elliptical { length, .. } => *length,
            Self::Parabolic { length, .. } => *length,
            Self::PowerSeries { length, .. } => *length,
            Self::Haack { length, .. } => *length,
        }
    }
}

impl MissileGeometryConfig {
    /// Evaluates the center of gravity curve at the given time
    pub fn current_cg(&self, time: Time) -> Vector3<Length> {
        let points = &self.cg_curve;
        if points.is_empty() {
            return Vector3::new(Length::default(), Length::default(), Length::default());
        }

        let t = time.get::<second>();
        let first_t = points[0].0.get::<second>();
        let last_t = points.last().unwrap().0.get::<second>();

        if t <= first_t {
            return points[0].1;
        }
        if t >= last_t {
            return points.last().unwrap().1;
        }

        for i in 0..points.len() - 1 {
            let t0 = points[i].0.get::<second>();
            let t1 = points[i + 1].0.get::<second>();

            if t >= t0 && t <= t1 {
                let cg0 = points[i].1;
                let cg1 = points[i + 1].1;
                let percent = (t - t0) / (t1 - t0);
                return cg0 + (cg1 - cg0).map(|len| len * percent);
            }
        }

        points.last().unwrap().1
    }
}

impl MissileEngineConfig {
    /// Evaluates the configured motor impulse curve at the given time
    pub fn current_thrust(&self, time: Time) -> Force {
        let points = &self.motor_impulse_curve;
        if points.is_empty() {
            return Force::new::<uom::si::force::newton>(0.0);
        }

        let t = time.get::<second>();
        let first_t = points[0].0.get::<second>();
        let last_t = points.last().unwrap().0.get::<second>();

        if t <= first_t {
            return points[0].1;
        }
        if t >= last_t {
            return points.last().unwrap().1;
        }

        for i in 0..points.len() - 1 {
            let t0 = points[i].0.get::<second>();
            let t1 = points[i + 1].0.get::<second>();

            if t >= t0 && t <= t1 {
                let f0 = points[i].1;
                let f1 = points[i + 1].1;
                let percent = (t - t0) / (t1 - t0);
                return f0 + (f1 - f0) * percent;
            }
        }

        Force::new::<uom::si::force::newton>(0.0)
    }
}

impl MissileMassConfig {
    /// Evaluates the inertia tensor curve at the given time
    pub fn current_inertia_tensor(&self, time: Time) -> Matrix3<f64> {
        let points = &self.inertia_tensor_curve;
        if points.is_empty() {
            return Matrix3::zeros();
        }

        let t = time.get::<second>();
        let first_t = points[0].0.get::<second>();
        let last_t = points.last().unwrap().0.get::<second>();

        if t <= first_t {
            return points[0].1;
        }
        if t >= last_t {
            return points.last().unwrap().1;
        }

        for i in 0..points.len() - 1 {
            let t0 = points[i].0.get::<second>();
            let t1 = points[i + 1].0.get::<second>();

            if t >= t0 && t <= t1 {
                let i0 = points[i].1;
                let i1 = points[i + 1].1;
                let percent = (t - t0) / (t1 - t0);
                return i0 + (i1 - i0) * percent;
            }
        }

        points.last().unwrap().1
    }

    /// Evaluates the configured mass curve at the given time
    pub fn current_mass(&self, time: Time) -> Mass {
        let points = &self.mass_curve;
        if points.is_empty() {
            return self.dry_mass;
        }

        let t = time.get::<second>();
        let first_t = points[0].0.get::<second>();
        let last_t = points.last().unwrap().0.get::<second>();

        if t <= first_t {
            return points[0].1;
        }
        if t >= last_t {
            return points.last().unwrap().1;
        }

        for i in 0..points.len() - 1 {
            let t0 = points[i].0.get::<second>();
            let t1 = points[i + 1].0.get::<second>();

            if t >= t0 && t <= t1 {
                let m0 = points[i].1;
                let m1 = points[i + 1].1;
                let percent = (t - t0) / (t1 - t0);
                return m0 + (m1 - m0) * percent;
            }
        }

        self.dry_mass
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;
    use uom::si::f64::{Length, Time};
    use uom::si::length::meter;
    use uom::si::time::second;

    fn default_geom() -> MissileGeometryConfig {
        MissileGeometryConfig {
            nosecone_shape: NoseconeShape::Ogive {
                length: Length::new::<meter>(0.2),
                blunting_radius: None,
                secant_radius: None,
            },
            cylindrical_body_length: Length::new::<meter>(1.0),
            diameter: Length::new::<meter>(0.1),
            fin_set: FinGeometry {
                num_fins: 4,
                offset_from_nose: Length::new::<meter>(0.9),
                root_chord: Length::new::<meter>(0.1),
                tip_chord: Length::new::<meter>(0.05),
                span: Length::new::<meter>(0.15),
                sweep_length: Length::new::<meter>(0.02),
                thickness: Length::new::<meter>(0.005),
                leading_edge_profile: FinEdgeProfile::Straight,
                trailing_edge_profile: FinEdgeProfile::Straight,
                edge_chamfer: Length::new::<meter>(0.0),
            },
            cg_curve: vec![],
        }
    }

    #[test]
    fn test_current_cg_empty() {
        let geom = default_geom();
        let cg = geom.current_cg(Time::new::<second>(5.0));
        assert_eq!(cg.x.value, 0.0);
        assert_eq!(cg.y.value, 0.0);
        assert_eq!(cg.z.value, 0.0);
    }

    #[test]
    fn test_current_cg_before_first() {
        let mut geom = default_geom();
        geom.cg_curve = vec![
            (
                Time::new::<second>(2.0),
                Vector3::new(
                    Length::new::<meter>(1.0),
                    Length::new::<meter>(2.0),
                    Length::new::<meter>(3.0),
                ),
            ),
            (
                Time::new::<second>(4.0),
                Vector3::new(
                    Length::new::<meter>(4.0),
                    Length::new::<meter>(5.0),
                    Length::new::<meter>(6.0),
                ),
            ),
        ];
        let cg = geom.current_cg(Time::new::<second>(1.0));
        assert_eq!(cg.x.value, 1.0);
        assert_eq!(cg.y.value, 2.0);
        assert_eq!(cg.z.value, 3.0);
    }

    #[test]
    fn test_current_cg_after_last() {
        let mut geom = default_geom();
        geom.cg_curve = vec![
            (
                Time::new::<second>(2.0),
                Vector3::new(
                    Length::new::<meter>(1.0),
                    Length::new::<meter>(2.0),
                    Length::new::<meter>(3.0),
                ),
            ),
            (
                Time::new::<second>(4.0),
                Vector3::new(
                    Length::new::<meter>(4.0),
                    Length::new::<meter>(5.0),
                    Length::new::<meter>(6.0),
                ),
            ),
        ];
        let cg = geom.current_cg(Time::new::<second>(5.0));
        assert_eq!(cg.x.value, 4.0);
        assert_eq!(cg.y.value, 5.0);
        assert_eq!(cg.z.value, 6.0);
    }

    #[test]
    fn test_current_cg_interpolation() {
        let mut geom = default_geom();
        geom.cg_curve = vec![
            (
                Time::new::<second>(2.0),
                Vector3::new(
                    Length::new::<meter>(10.0),
                    Length::new::<meter>(20.0),
                    Length::new::<meter>(30.0),
                ),
            ),
            (
                Time::new::<second>(4.0),
                Vector3::new(
                    Length::new::<meter>(20.0),
                    Length::new::<meter>(30.0),
                    Length::new::<meter>(40.0),
                ),
            ),
        ];
        let cg = geom.current_cg(Time::new::<second>(3.0));
        assert_eq!(cg.x.value, 15.0);
        assert_eq!(cg.y.value, 25.0);
        assert_eq!(cg.z.value, 35.0);
    }

    fn default_mass() -> MissileMassConfig {
        use uom::si::mass::kilogram;

        MissileMassConfig {
            dry_mass: uom::si::f64::Mass::new::<kilogram>(5.0),
            wet_mass: uom::si::f64::Mass::new::<kilogram>(10.0),
            mass_curve: vec![],
            inertia_tensor_curve: vec![],
        }
    }

    #[test]
    fn test_current_inertia_tensor_interpolation() {
        use nalgebra::Matrix3;

        let mut mass = default_mass();
        mass.inertia_tensor_curve = vec![
            (
                Time::new::<second>(0.0),
                Matrix3::new(1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0),
            ),
            (
                Time::new::<second>(2.0),
                Matrix3::new(3.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 5.0),
            ),
        ];

        let tensor_mid = mass.current_inertia_tensor(Time::new::<second>(1.0));
        assert_eq!(tensor_mid[(0, 0)], 2.0);
        assert_eq!(tensor_mid[(1, 1)], 3.0);
        assert_eq!(tensor_mid[(2, 2)], 4.0);

        let tensor_late = mass.current_inertia_tensor(Time::new::<second>(3.0));
        assert_eq!(tensor_late[(0, 0)], 3.0);
        assert_eq!(tensor_late[(1, 1)], 4.0);
        assert_eq!(tensor_late[(2, 2)], 5.0);
    }
}

impl MissileControllerConfig {
    fn interpolate_gain(points: &Vec<(Time, f64)>, time: Time) -> f64 {
        if points.is_empty() {
            return 0.0;
        }

        let t = time.get::<second>();
        let first_t = points[0].0.get::<second>();
        let last_t = points.last().unwrap().0.get::<second>();

        if t <= first_t {
            return points[0].1;
        }
        if t >= last_t {
            return points.last().unwrap().1;
        }

        for i in 0..points.len() - 1 {
            let t0 = points[i].0.get::<second>();
            let t1 = points[i + 1].0.get::<second>();

            if t >= t0 && t <= t1 {
                let g0 = points[i].1;
                let g1 = points[i + 1].1;
                let percent = (t - t0) / (t1 - t0);
                return g0 + (g1 - g0) * percent;
            }
        }
        points.last().unwrap().1
    }

    /// Linearly interpolates the scheduled pitch Kp gain based on flight time.
    pub fn current_pitch_kp(&self, time: Time) -> f64 {
        Self::interpolate_gain(&self.pitch_pi_kp, time)
    }

    /// Linearly interpolates the scheduled pitch Ki gain based on flight time.
    pub fn current_pitch_ki(&self, time: Time) -> f64 {
        Self::interpolate_gain(&self.pitch_pi_ki, time)
    }

    /// Linearly interpolates the scheduled yaw Kp gain based on flight time.
    pub fn current_yaw_kp(&self, time: Time) -> f64 {
        Self::interpolate_gain(&self.yaw_pi_kp, time)
    }

    /// Linearly interpolates the scheduled yaw Ki gain based on flight time.
    pub fn current_yaw_ki(&self, time: Time) -> f64 {
        Self::interpolate_gain(&self.yaw_pi_ki, time)
    }
}
