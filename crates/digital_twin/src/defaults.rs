use digital_twin_glue::prelude::*;
use nalgebra::{Matrix3, UnitQuaternion, Vector3};
use uom::si::angle::radian;
use uom::si::angular_velocity::radian_per_second;
use uom::si::f64::{Angle, AngularVelocity, Force, Length, Mass, Time, Velocity};
use uom::si::force::newton;
use uom::si::length::meter;
use uom::si::mass::kilogram;
use uom::si::time::second;
use uom::si::velocity::meter_per_second;

pub fn get_initial_state(config: &MissileConfig) -> MissileState {
    MissileState {
        position: [
            Length::new::<meter>(0.0),
            Length::new::<meter>(0.0),
            Length::new::<meter>(500.0),
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
        mass: MissileMassConfig {
            dry_mass: Mass::new::<kilogram>(40.0),
            wet_mass: Mass::new::<kilogram>(140.0),
            mass_curve: vec![
                (Time::new::<second>(0.0), Mass::new::<kilogram>(140.0)),
                (Time::new::<second>(10.4), Mass::new::<kilogram>(40.0)),
                (Time::new::<second>(100.0), Mass::new::<kilogram>(40.0)),
            ],
            // Normalized inertia tensor (I / m) for a cylinder: L = 1.4m, r = 0.05m
            // I_xx = I_yy = (3*r^2 + L^2) / 12 = 0.1639
            // I_zz = (r^2) / 2 = 0.00125
            inertia_tensor_curve: vec![(
                Time::new::<second>(0.0),
                Matrix3::new(0.1639, 0.0, 0.0, 0.0, 0.1639, 0.0, 0.0, 0.0, 0.00125),
            )],
        },
        geometry: MissileGeometryConfig {
            body_length: Length::new::<meter>(1.4),
            diameter: Length::new::<meter>(0.1),
            fin_offset_from_nose: Length::new::<meter>(1.2),
            fin_chord_length: Length::new::<meter>(0.2),
            cg_curve: vec![
                (
                    Time::new::<second>(0.0),
                    Vector3::new(
                        Length::new::<meter>(0.0),
                        Length::new::<meter>(0.0),
                        Length::new::<meter>(-0.25),
                    ),
                ),
                (
                    Time::new::<second>(10.4),
                    Vector3::new(
                        Length::new::<meter>(0.0),
                        Length::new::<meter>(0.0),
                        Length::new::<meter>(0.0),
                    ),
                ),
                (
                    Time::new::<second>(100.0),
                    Vector3::new(
                        Length::new::<meter>(0.0),
                        Length::new::<meter>(0.0),
                        Length::new::<meter>(0.0),
                    ),
                ),
            ],
        },
        controller: MissileControllerConfig {
            pitch_pid_kp: 0.365516,
            pitch_pid_ki: 0.100000,
            pitch_pid_kd: 0.000000,
            yaw_pid_kp: 0.365516,
            yaw_pid_ki: 0.100000,
            yaw_pid_kd: 0.000000,
        },
        engine: MissileEngineConfig {
            motor_impulse_curve: vec![
                (Time::new::<second>(0.0), Force::new::<newton>(0.0)),
                (Time::new::<second>(0.1), Force::new::<newton>(17800.0)),
                (Time::new::<second>(10.2), Force::new::<newton>(17800.0)),
                (Time::new::<second>(10.4), Force::new::<newton>(0.0)),
            ],
            max_tvc_angle: Angle::new::<radian>(20.0_f64.to_radians()),
        },
    }
}
