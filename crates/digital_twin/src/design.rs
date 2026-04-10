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

pub fn get_initial_state(config: &MissileConfig) -> MissileState {
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

pub fn get_default_config() -> MissileConfig {
    MissileConfig {
        environment: EnvironmentalConfig::default(),
        mass: MissileMassConfig {
            dry_mass: Mass::new::<kilogram>(40.0),
            wet_mass: Mass::new::<kilogram>(140.0),
            mass_curve: vec![
                (Time::new::<second>(0.0), Mass::new::<kilogram>(140.0)),
                (Time::new::<second>(10.4), Mass::new::<kilogram>(40.0)),
                (Time::new::<second>(100.0), Mass::new::<kilogram>(40.0)),
            ],
            inertia_tensor_curve: vec![
                (
                    Time::new::<second>(0.0),
                    Matrix3::new(30.00, 0.0, 0.0, 0.0, 30.00, 0.0, 0.0, 0.0, 0.180),
                ),
                (
                    Time::new::<second>(2.6),
                    Matrix3::new(26.71, 0.0, 0.0, 0.0, 26.71, 0.0, 0.0, 0.0, 0.147),
                ),
                (
                    Time::new::<second>(5.2),
                    Matrix3::new(22.74, 0.0, 0.0, 0.0, 22.74, 0.0, 0.0, 0.0, 0.115),
                ),
                (
                    Time::new::<second>(7.8),
                    Matrix3::new(17.32, 0.0, 0.0, 0.0, 17.32, 0.0, 0.0, 0.0, 0.082),
                ),
                (
                    Time::new::<second>(10.4),
                    Matrix3::new(7.70, 0.0, 0.0, 0.0, 7.70, 0.0, 0.0, 0.0, 0.050),
                ),
                (
                    Time::new::<second>(100.0),
                    Matrix3::new(7.70, 0.0, 0.0, 0.0, 7.70, 0.0, 0.0, 0.0, 0.050),
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
                        Length::new::<meter>(-0.1000),
                    ),
                ),
                (
                    Time::new::<second>(2.6),
                    Vector3::new(
                        Length::new::<meter>(0.0),
                        Length::new::<meter>(0.0),
                        Length::new::<meter>(-0.0565),
                    ),
                ),
                (
                    Time::new::<second>(5.2),
                    Vector3::new(
                        Length::new::<meter>(0.0),
                        Length::new::<meter>(0.0),
                        Length::new::<meter>(0.0111),
                    ),
                ),
                (
                    Time::new::<second>(7.8),
                    Vector3::new(
                        Length::new::<meter>(0.0),
                        Length::new::<meter>(0.0),
                        Length::new::<meter>(0.1308),
                    ),
                ),
                (
                    Time::new::<second>(10.4),
                    Vector3::new(
                        Length::new::<meter>(0.0),
                        Length::new::<meter>(0.0),
                        Length::new::<meter>(0.4000),
                    ),
                ),
                (
                    Time::new::<second>(100.0),
                    Vector3::new(
                        Length::new::<meter>(0.0),
                        Length::new::<meter>(0.0),
                        Length::new::<meter>(0.4000),
                    ),
                ),
            ],
        },
        controller: MissileControllerConfig {
            pitch_pid_kp: vec![(Time::new::<second>(0.0), 0.642419)],
            pitch_pid_ki: vec![(Time::new::<second>(0.0), 0.000000)],
            pitch_pid_kd: vec![(Time::new::<second>(0.0), 0.05)],
            yaw_pid_kp: vec![(Time::new::<second>(0.0), 0.642419)],
            yaw_pid_ki: vec![(Time::new::<second>(0.0), 0.000000)],
            yaw_pid_kd: vec![(Time::new::<second>(0.0), 0.05)],
        },
        engine: MissileEngineConfig {
            motor_impulse_curve: vec![
                (Time::new::<second>(0.0), Force::new::<newton>(0.0)),
                (Time::new::<second>(0.1), Force::new::<newton>(17800.0)),
                (Time::new::<second>(10.2), Force::new::<newton>(17800.0)),
                (Time::new::<second>(10.4), Force::new::<newton>(0.0)),
            ],
            max_tvc_angle: Angle::new::<degree>(20.0),
            tvc_slew_rate: AngularVelocity::new::<degree_per_second>(60.0),
            tvc_activation_delay: Time::new::<millisecond>(50.0),
        },
    }
}
