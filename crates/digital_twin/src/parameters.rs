use digital_twin_glue::prelude::*;
use nalgebra::{Matrix3, UnitQuaternion, Vector3};
use uom::si::angle::{degree, radian};
use uom::si::angular_velocity::{degree_per_second, radian_per_second};
use uom::si::f64::{Angle, AngularVelocity, Force, Length, Mass, Time, Velocity};
use uom::si::force::newton;
use uom::si::length::{foot, inch, meter};
use uom::si::mass::kilogram;
use uom::si::time::{millisecond, second};
use uom::si::velocity::meter_per_second;

pub fn get_rocket_initial_state(config: &MissileConfig) -> MissileState {
    MissileState {
        position: [
            Length::new::<meter>(0.0),
            Length::new::<meter>(0.0),
            Length::new::<meter>(10.0),
        ]
        .into(),
        body_velocity: [
            Velocity::new::<meter_per_second>(0.0),
            Velocity::new::<meter_per_second>(0.0),
            Velocity::new::<meter_per_second>(0.0),
        ]
        .into(),
        orientation: UnitQuaternion::identity(),
        angular_velocity: [
            AngularVelocity::new::<radian_per_second>(0.0),
            AngularVelocity::new::<radian_per_second>(0.0),
            AngularVelocity::new::<radian_per_second>(0.0),
        ]
        .into(),
        fin_angles: [Angle::new::<radian>(0.0); 4],
        tvc_angles: [Angle::new::<radian>(0.0); 2],
        time: Time::new::<second>(0.0),
        current_mass: config.mass.wet_mass,
        motor_thrust: Force::new::<newton>(0.0),
        inertia_tensor: config.mass.current_inertia_tensor(Time::new::<second>(0.0)),
    }
}

pub fn get_rocket_design() -> MissileConfig {
    MissileConfig {
        environment: EnvironmentalConfig::default(),
        mass: MissileMassConfig {
            dry_mass: Mass::new::<kilogram>(9.0),
            wet_mass: Mass::new::<kilogram>(50.0),
            mass_curve: vec![
                (Time::new::<second>(0.0), Mass::new::<kilogram>(50.0)),
                (Time::new::<second>(3.0), Mass::new::<kilogram>(29.5)),
                (Time::new::<second>(6.0), Mass::new::<kilogram>(9.0)),
                (Time::new::<second>(100.0), Mass::new::<kilogram>(9.0)),
            ],
            inertia_tensor_curve: vec![
                (
                    Time::new::<second>(0.0),
                    Matrix3::new(9.70, 0.0, 0.0, 0.0, 9.70, 0.0, 0.0, 0.0, 0.064),
                ),
                (
                    Time::new::<second>(1.5),
                    Matrix3::new(7.71, 0.0, 0.0, 0.0, 7.71, 0.0, 0.0, 0.0, 0.051),
                ),
                (
                    Time::new::<second>(3.0),
                    Matrix3::new(5.73, 0.0, 0.0, 0.0, 5.73, 0.0, 0.0, 0.0, 0.038),
                ),
                (
                    Time::new::<second>(4.5),
                    Matrix3::new(3.74, 0.0, 0.0, 0.0, 3.74, 0.0, 0.0, 0.0, 0.024),
                ),
                (
                    Time::new::<second>(6.0),
                    Matrix3::new(1.75, 0.0, 0.0, 0.0, 1.75, 0.0, 0.0, 0.0, 0.011),
                ),
                (
                    Time::new::<second>(100.0),
                    Matrix3::new(1.75, 0.0, 0.0, 0.0, 1.75, 0.0, 0.0, 0.0, 0.011),
                ),
            ],
        },
        geometry: MissileGeometryConfig {
            nosecone_shape: NoseconeShape::Ogive {
                length: Length::new::<inch>(8.0),
                blunting_radius: Some(Length::new::<inch>(0.1)),
                secant_radius: None,
            },
            cylindrical_body_length: Length::new::<foot>(5.0),
            diameter: Length::new::<inch>(4.0),
            fin_set: FinGeometry {
                num_fins: 4,
                offset_from_nose: Length::new::<foot>(4.5),
                root_chord: Length::new::<inch>(8.0),
                tip_chord: Length::new::<inch>(4.0),
                span: Length::new::<inch>(8.0),
                sweep_length: Length::new::<inch>(2.0),
                thickness: Length::new::<inch>(0.2),
                leading_edge_profile: FinEdgeProfile::Straight,
                trailing_edge_profile: FinEdgeProfile::Straight,
                edge_chamfer: Length::new::<inch>(0.078),
            },
            cg_curve: vec![
                (
                    Time::new::<second>(0.0),
                    Vector3::new(
                        Length::new::<meter>(0.0),
                        Length::new::<meter>(0.0),
                        Length::new::<meter>(-0.2000),
                    ),
                ),
                (
                    Time::new::<second>(3.0),
                    Vector3::new(
                        Length::new::<meter>(0.0),
                        Length::new::<meter>(0.0),
                        Length::new::<meter>(0.0500),
                    ),
                ),
                (
                    Time::new::<second>(6.0),
                    Vector3::new(
                        Length::new::<meter>(0.0),
                        Length::new::<meter>(0.0),
                        Length::new::<meter>(0.3000),
                    ),
                ),
                (
                    Time::new::<second>(100.0),
                    Vector3::new(
                        Length::new::<meter>(0.0),
                        Length::new::<meter>(0.0),
                        Length::new::<meter>(0.3000),
                    ),
                ),
            ],
        },
        controller: MissileControllerConfig {
            pitch_pid_kp: vec![
                (Time::new::<second>(0.0), 0.985447),
                (Time::new::<second>(5.2), 0.967288),
                (Time::new::<second>(10.4), 0.882278),
            ],
            pitch_pid_ki: vec![
                (Time::new::<second>(0.0), 0.294754),
                (Time::new::<second>(5.2), 0.300649),
                (Time::new::<second>(10.4), 0.284024),
            ],
            pitch_pid_kd: vec![
                (Time::new::<second>(0.0), 0.050834),
                (Time::new::<second>(5.2), 0.063916),
                (Time::new::<second>(10.4), 0.047683),
            ],
            yaw_pid_kp: vec![
                (Time::new::<second>(0.0), 0.985447),
                (Time::new::<second>(5.2), 0.967288),
                (Time::new::<second>(10.4), 0.882278),
            ],
            yaw_pid_ki: vec![
                (Time::new::<second>(0.0), 0.294754),
                (Time::new::<second>(5.2), 0.300649),
                (Time::new::<second>(10.4), 0.284024),
            ],
            yaw_pid_kd: vec![
                (Time::new::<second>(0.0), 0.050834),
                (Time::new::<second>(5.2), 0.063916),
                (Time::new::<second>(10.4), 0.047683),
            ],
        },
        engine: MissileEngineConfig {
            motor_impulse_curve: vec![
                (Time::new::<second>(0.0), Force::new::<newton>(0.0)),
                (Time::new::<second>(0.1), Force::new::<newton>(15500.0)),
                (Time::new::<second>(3.0), Force::new::<newton>(14000.0)),
                (Time::new::<second>(5.8), Force::new::<newton>(13000.0)),
                (Time::new::<second>(6.0), Force::new::<newton>(0.0)),
            ],
            max_tvc_angle: Angle::new::<degree>(20.0),
            tvc_slew_rate: AngularVelocity::new::<degree_per_second>(60.0),
            tvc_activation_delay: Time::new::<millisecond>(50.0),
        },
    }
}
