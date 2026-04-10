use crate::control::rocket::Rocket;
use crate::control::state::MissileState;
use crate::geometry::config::MissileConfig;
use crate::geometry::mesh::{Mesh, MeshGenerator};
use crate::integ;
use crate::physics::solver::{AeroSolver, lookup_atmosphere};
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
    pub rocket: Box<dyn Rocket>,
    pub solver: AeroSolver,
    pub mesh_generator: Box<dyn MeshGenerator>,
    pub current_mesh: Mesh,
    rk4: integ::rk4::Rk4,
}

impl DigitalTwin {
    pub fn new(
        config: MissileConfig,
        state: MissileState,
        mesh_generator: Box<dyn MeshGenerator>,
        rocket: Box<dyn Rocket>,
    ) -> Self {
        Self {
            config,
            state,
            solver: AeroSolver {},
            mesh_generator,
            rocket,
            current_mesh: Mesh::default(),
            rk4: integ::rk4::Rk4 {},
        }
    }

    pub fn step(&mut self, dt: f64) {
        // 1. Update Rocket first (to get new fin angles/thrust settings)
        // The FC sets the "intent" for the next step.
        self.state = self.rocket.update(&self.state, dt);

        // Update mesh once per step instead of inside the RK4 loop
        self.mesh_generator
            .generate(&self.state, &self.config, &mut self.current_mesh);
        // Precompute normals/centroids so we don't recalculate 4 times per RK4 step
        self.current_mesh.compute_surface_properties();

        // Lookup atmospheric properties once per step
        let altitude = self.state.position.z.value.abs();
        let (air_density, speed_of_sound, dyn_viscosity) = lookup_atmosphere(altitude);

        // 2. Define the "System Dynamics"
        let t_sec = self.state.time.value;
        let mut sub_state = self.state.clone();
        let physics_engine =
            |v: Vector3<f64>, w: Vector3<f64>, q: Quaternion<f64>, p: Vector3<f64>| {
                // Map raw f64s back into uom quantities
                sub_state.position = p.map(Length::new::<meter>);
                sub_state.body_velocity = v.map(Velocity::new::<meter_per_second>);
                sub_state.angular_velocity = w.map(AngularVelocity::new::<radian_per_second>);
                sub_state.orientation = UnitQuaternion::from_quaternion(q);

                let turb_factor = self.config.environment.turbulence_intensity;
                let gust_x = turb_factor * ((t_sec * 2.1).sin() + 0.5 * (t_sec * 5.3 + 1.2).sin());
                let gust_y =
                    turb_factor * ((t_sec * 1.7 + 2.0).sin() + 0.5 * (t_sec * 4.1 + 0.5).sin());
                let gust_z = turb_factor * ((t_sec * 3.3 + 1.0).sin() * 0.3);

                let base_wind = self
                    .config
                    .environment
                    .wind_velocity
                    .map(|v| v.get::<meter_per_second>());
                let total_wind_f64 = base_wind + Vector3::new(gust_x, gust_y, gust_z);

                // Convert world-frame wind to body-frame and compute relative airspeed
                let wind_body_f64 = sub_state.orientation.conjugate() * total_wind_f64;
                let wind_body_vel = wind_body_f64.map(Velocity::new::<meter_per_second>);
                sub_state.body_velocity -= wind_body_vel;

                let output = self.solver.calculate_forces(
                    &self.current_mesh,
                    &sub_state,
                    air_density,
                    speed_of_sound,
                    dyn_viscosity,
                );

                // Restore absolute body velocity before continuing (RK4 needs true state velocity, not airspeed)
                sub_state.body_velocity += wind_body_vel;

                // Add active engine propulsion (Thrust + TVC alignment)
                let thrust_mag = sub_state.motor_thrust.get::<newton>();
                let tvc_pitch = sub_state.tvc_angles[0].get::<radian>();
                let tvc_yaw = sub_state.tvc_angles[1].get::<radian>();

                // Base thrust acts forward down the Z-axis of the vehicle body
                let thrust_base = Vector3::new(0.0, 0.0, thrust_mag);

                // TVC servos gimbal the nozzle rotating the thrust vector
                let tvc_rotation = Rotation3::from_euler_angles(tvc_pitch, tvc_yaw, 0.0);
                let active_thrust = tvc_rotation * thrust_base;

                // Fetch dynamic Center of Gravity (CoG) from curve
                let cg = self.config.geometry.current_cg(sub_state.time);
                let body_length = self.config.geometry.cylindrical_body_length.get::<meter>();
                let nose_length = self.config.geometry.nosecone_shape.length().get::<meter>();
                let total_length = body_length + nose_length;

                // Introduce rotational torque from off-axis thrust relative to current CoG
                // Nozzle is at the base of the rocket (-total_length/2.0 from geometric center)
                // Offset is relative to geometric center, so subtract cg to get offset from cg
                let nozzle_offset =
                    Vector3::new(0.0, 0.0, -total_length / 2.0) - cg.map(|c| c.get::<meter>());
                let motor_torque = nozzle_offset.cross(&active_thrust);

                (output.force + active_thrust, output.torque + motor_torque)
            };

        // 3. Let RK4 run the show
        self.state = self.rk4.step(&self.state, physics_engine, dt);
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
