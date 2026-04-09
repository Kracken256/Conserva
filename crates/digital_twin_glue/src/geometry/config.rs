use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};
use uom::si::f64::{Force, Length, Mass, Time};
use uom::si::time::second;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissileGeometryConfig {
    pub body_length: Length,
    pub diameter: Length,
    pub fin_offset_from_nose: Length,
    pub fin_chord_length: Length,
    pub cg_curve: Vec<(Time, Vector3<Length>)>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissileControllerConfig {
    pub pitch_pid_kp: f64,
    pub pitch_pid_ki: f64,
    pub pitch_pid_kd: f64,
    pub yaw_pid_kp: f64,
    pub yaw_pid_ki: f64,
    pub yaw_pid_kd: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissileEngineConfig {
    pub motor_impulse_curve: Vec<(Time, Force)>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissileMassConfig {
    pub dry_mass: Mass,
    pub wet_mass: Mass,
    pub mass_curve: Vec<(Time, Mass)>,
    pub inertia_tensor: Matrix3<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissileConfig {
    pub geometry: MissileGeometryConfig,
    pub controller: MissileControllerConfig,
    pub engine: MissileEngineConfig,
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
}
