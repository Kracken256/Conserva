use digital_twin_glue::prelude::*;
use nalgebra::{Matrix3, UnitQuaternion};
use uom::si::angle::radian;
use uom::si::angular_velocity::radian_per_second;
use uom::si::f32::{Angle, AngularVelocity, Force, Length, Mass, Time, Velocity};
use uom::si::force::newton;
use uom::si::length::meter;
use uom::si::mass::kilogram;
use uom::si::time::second;
use uom::si::velocity::meter_per_second;

pub fn get_initial_state() -> MissileState {
    MissileState {
        position: [
            Length::new::<meter>(0.0),
            Length::new::<meter>(0.0),
            Length::new::<meter>(0.0),
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
        mass: Mass::new::<kilogram>(100.0),
        inertia_tensor: Matrix3::identity(),
    }
}

pub fn get_default_config() -> MissileConfig {
    MissileConfig {
        body_length: Length::new::<meter>(1.4),
        diameter: Length::new::<meter>(0.1),
        fin_offset_from_nose: Length::new::<meter>(0.2),
        fin_chord_length: Length::new::<meter>(0.05),
        motor_impulse_curve: vec![
            (Time::new::<second>(0.0), Force::new::<newton>(0.0)),
            (Time::new::<second>(1.0), Force::new::<newton>(1.0)),
        ],
    }
}
