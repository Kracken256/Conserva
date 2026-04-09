use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};
use uom::si::f64::{Angle, Force, Length, Mass, Time};
use uom::si::time::second;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissileGeometryConfig {
    /// The total length of the main cylindrical body of the missile. It defines the primary
    /// aerodynamic acting surface and total volume. This measurement actively dictates the
    /// dimensional space available for the payload, electronics, and propellant.
    pub body_length: Length,
    /// The maximum diameter of the missile's main body tube. This measurement bounds the internal
    /// volume and dictates the frontal reference area utilized in aerodynamic drag estimations. It
    /// restricts component sizing throughout the chassis.
    pub diameter: Length,
    /// The distance measured from the very tip of the missile's nose to the leading edge of the
    /// fins at their root. This placement is highly influential on the aerodynamic center of
    /// pressure. Shifting this surface aft generally increases passive stability bounds.
    pub fin_offset_from_nose: Length,
    /// The physical length of the fin's root edge where it structurally attaches to the main
    /// external aerodynamic body. This dictates the geometric span of the control surface in the
    /// longitudinal axis. Longer chord distances generally provide a broader surface area for
    /// control authority.
    pub fin_chord_length: Length,
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
    pub pitch_pid_kp: f64,
    /// The integral gain tuning constant for the automated pitch control loop. It aggregates and
    /// counters accumulated error histories to wash out steady-state offsets—like persistent
    /// gravity sag. However, an excessively large value often manifests as delayed overshoots and
    /// integral windups.
    pub pitch_pid_ki: f64,
    /// The derivative gain scaling factor governing the pitch control loop dampening. It estimates
    /// future errors by observing the rate of change of the immediate pitch displacement,
    /// preemptively braking rapid orientation shifts. This critical damping component aids in
    /// smoothing out rapid flight maneuvers.
    pub pitch_pid_kd: f64,
    /// The unscaled proportional gain factor controlling the horizontal thrust vector control and
    /// fin yaw loop. It produces immediate physical reactions when the current telemetry heading
    /// deviates from the navigational target heading. Proper tuning tightens flight tracking
    /// capabilities against active lateral constraints.
    pub yaw_pid_kp: f64,
    /// The foundational integral multiplier backing the horizontal yaw control loop tuning. It
    /// steadily ramps up correction commands if the heading persistently lingers off-band due to
    /// unresolved continuous disturbances. This function acts primarily to negate sustained
    /// environmental crosswinds over an extended trajectory.
    pub yaw_pid_ki: f64,
    /// The primary derivative scaling value managing the horizontal yaw correction rate dampening.
    /// It sharply reacts to the immediate velocity of the yaw error variation to heavily cushion
    /// rotational movement and prevent rapid heading overshoot snaps. This stabilizing force
    /// permits rapid horizontal intercept corrections without destructive fishtails.
    pub yaw_pid_kd: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissileEngineConfig {
    /// An interpolated time-series array establishing the engine's functional thrust mapping
    /// profile over its comprehensive burn span. Each vertex binds a timestamp to the maximum total
    /// force the exhaust plume exerts on the airframe. The completion of this trace fundamentally
    /// indicates motor burnout and aerodynamic coasting operations.
    pub motor_impulse_curve: Vec<(Time, Force)>,
    /// The maximum physical deflection angle allowed by the Thrust Vector Control (TVC) nozzle
    /// gimbal actuators. This hard constraint limits the magnitude of correcting moments the
    /// engine can generate in a single tick regardless of PID requests. Proper tuning ensures
    /// the guidance software respects physical hardware servo limits without commanding over-travel.
    pub max_tvc_angle: Angle,
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
    /// An interpolated time-series mapping that tracks the dynamic 3x3 inertia tensor matrix over the
    /// duration of the motor burn. As solid propellant mass depletes, the vehicle's structural
    /// resistance to pitch, yaw, and roll torques constantly shifts. The physics engine continually
    /// evaluates this curve to accurately model changing rotational kinetics and moment distributions
    /// throughout the entire flight envelope.
    pub inertia_tensor_curve: Vec<(Time, Matrix3<f64>)>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissileConfig {
    /// Gathers all defining structural boundary measurements obligatory for constructing the three-
    /// dimensional airframe chassis. This envelope fundamentally incorporates base diameters
    /// alongside length offsets essential for assembling realistic geometry structures.
    /// Consequently, it dominates the resulting baseline estimations mapped during physics solver
    /// friction integration.
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
    /// Manages the fundamental kinematic state constraints, including the inert bounds spanning the
    /// peak physical wet deployment masses down through totally depleted dry casings. It dictates
    /// the structural moment distribution tracking across non-linear transient burn envelopes
    /// seamlessly synchronized within physics derivations. The rigid rotational matrix tensors
    /// bundled strictly enforce handling rigidity constraints applied against the physics engine
    /// torques.
    pub mass: MissileMassConfig,
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
            body_length: Length::new::<meter>(1.0),
            diameter: Length::new::<meter>(0.1),
            fin_offset_from_nose: Length::new::<meter>(0.9),
            fin_chord_length: Length::new::<meter>(0.1),
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
