use crate::prelude::*;
use nalgebra::{Quaternion, Rotation3, UnitQuaternion, Vector3};
use uom::si::angle::radian;
use uom::si::angular_velocity::{AngularVelocity, radian_per_second};
use uom::si::f64::{Length, Velocity};
use uom::si::force::newton;
use uom::si::length::meter;
use uom::si::velocity::meter_per_second;

pub struct DigitalTwin {
    pub config: MissileConfig,
    pub state: MissileState,
    pub rocket: Box<dyn RocketCtrl>,
    pub solver: AeroSolver,
    pub mesh_generator: Box<dyn MeshGenerator>,
    pub current_mesh: Mesh,
    rk4: crate::integ::Rk4,
}

impl DigitalTwin {
    pub fn new(
        config: MissileConfig,
        state: MissileState,
        mesh_generator: Box<dyn MeshGenerator>,
        rocket: Box<dyn RocketCtrl>,
    ) -> Self {
        Self {
            config,
            state,
            solver: AeroSolver {},
            mesh_generator,
            rocket,
            current_mesh: Mesh::default(),
            rk4: crate::integ::Rk4 {},
        }
    }

    pub fn step(&mut self, dt: f64) {
        self.update_flight_computer(dt);
        self.update_aerodynamic_mesh();

        let (air_density, speed_of_sound, dyn_viscosity) = self.lookup_current_atmosphere();
        
        let t_sec = self.state.time.value;
        let mut sub_state = self.state.clone();

        let physics_engine = |v: Vector3<f64>, w: Vector3<f64>, q: Quaternion<f64>, p: Vector3<f64>| {
            Self::update_state_kinematics(&mut sub_state, p, v, w, q);

            let wind_body_vel = Self::compute_body_wind(&self.config, t_sec, &sub_state.orientation);
            
            // Apply wind as a relative velocity for aerodynamics
            sub_state.body_velocity -= wind_body_vel;
            let aero = self.solver.calculate_forces(
                &self.current_mesh,
                &sub_state,
                air_density,
                speed_of_sound,
                dyn_viscosity,
            );
            // Restore absolute ground-relative velocity
            sub_state.body_velocity += wind_body_vel;

            let (thrust, torque) = Self::compute_engine_dynamics(&self.config, &sub_state);

            (aero.force + thrust, aero.torque + torque)
        };

        self.state = self.rk4.step(&self.state, physics_engine, dt);
    }

    fn update_flight_computer(&mut self, dt: f64) {
        self.state = self.rocket.update(&self.state, dt);
    }

    fn update_aerodynamic_mesh(&mut self) {
        self.mesh_generator.generate(&self.state, &self.config, &mut self.current_mesh);
        self.current_mesh.compute_surface_properties();
    }

    fn lookup_current_atmosphere(&self) -> (f64, f64, f64) {
        let altitude = self.state.position.z.value.abs();
        lookup_atmosphere(altitude)
    }

    fn update_state_kinematics(
        state: &mut MissileState,
        p: Vector3<f64>,
        v: Vector3<f64>,
        w: Vector3<f64>,
        q: Quaternion<f64>,
    ) {
        state.position = p.map(Length::new::<meter>);
        state.body_velocity = v.map(Velocity::new::<meter_per_second>);
        state.angular_velocity = w.map(AngularVelocity::new::<radian_per_second>);
        state.orientation = UnitQuaternion::from_quaternion(q);
    }

    fn compute_body_wind(
        config: &MissileConfig,
        t_sec: f64,
        orientation: &UnitQuaternion<f64>,
    ) -> Vector3<Velocity> {
        let turb_factor = config.environment.turbulence_intensity.get::<meter_per_second>();
        let gust_x = turb_factor * ((t_sec * 2.1).sin() + 0.5 * (t_sec * 5.3 + 1.2).sin());
        let gust_y = turb_factor * ((t_sec * 1.7 + 2.0).sin() + 0.5 * (t_sec * 4.1 + 0.5).sin());
        let gust_z = turb_factor * ((t_sec * 3.3 + 1.0).sin() * 0.3);

        let base_wind = config.environment.wind_velocity.map(|v| v.get::<meter_per_second>());
        let total_wind = base_wind + Vector3::new(gust_x, gust_y, gust_z);

        // Convert world-frame wind to body-frame and compute relative airspeed
        let wind_body = orientation.conjugate() * total_wind;
        wind_body.map(Velocity::new::<meter_per_second>)
    }

    fn compute_engine_dynamics(
        config: &MissileConfig,
        state: &MissileState,
    ) -> (Vector3<f64>, Vector3<f64>) {
        let thrust_mag = state.motor_thrust.get::<newton>();
        let tvc_pitch = state.tvc_angles[0].get::<radian>();
        let tvc_yaw = state.tvc_angles[1].get::<radian>();

        let thrust_base = Vector3::new(0.0, 0.0, thrust_mag);
        let tvc_rotation = Rotation3::from_euler_angles(tvc_pitch, tvc_yaw, 0.0);
        let active_thrust = tvc_rotation * thrust_base;

        let cg = config.geometry.current_cg(state.time);
        let body_length = config.geometry.cylindrical_body_length.get::<meter>();
        let nose_length = config.geometry.nosecone_shape.length().get::<meter>();
        let total_length = body_length + nose_length;

        let nozzle_offset =
            Vector3::new(0.0, 0.0, -total_length / 2.0) - cg.map(|c| c.get::<meter>());
        let motor_torque = nozzle_offset.cross(&active_thrust);

        (active_thrust, motor_torque)
    }
}

#[cfg(test)]
mod tests {

    #[test]
    fn test_digital_twin_initialization_placeholder() {
        // Just asserting true here to have a test placeholder in digital_twin_glue
        assert!(true);
    }
}
