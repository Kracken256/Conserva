use crate::control::rocket::Rocket;
use crate::control::state::MissileState;
use crate::geometry::config::MissileConfig;
use crate::geometry::mesh::{Mesh, MeshGenerator};
use crate::integ;
use crate::physics::solver::AeroSolver;
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

        // 2. Define the "System Dynamics"
        let physics_engine =
            |v: &Vector3<f64>, w: &Vector3<f64>, q: &Quaternion<f64>, p: &Vector3<f64>| {
                // Create a temporary state for the sub-step
                let mut sub_state = self.state.clone();

                // Map raw f64s back into uom quantities
                sub_state.position = p.map(Length::new::<meter>);
                sub_state.body_velocity = v.map(Velocity::new::<meter_per_second>);
                sub_state.angular_velocity = w.map(AngularVelocity::new::<radian_per_second>);
                sub_state.orientation = UnitQuaternion::from_quaternion(*q);

                // Update mesh for this sub-step (now with altitude/position awareness!)
                self.mesh_generator
                    .generate(&sub_state, &self.config, &mut self.current_mesh);

                let output = self.solver.calculate_forces(&self.current_mesh, &sub_state);

                // Add active engine propulsion (Thrust + TVC alignment)
                let thrust_mag = sub_state.motor_thrust.get::<newton>();
                let tvc_pitch = sub_state.tvc_angles[0].get::<radian>();
                let tvc_yaw = sub_state.tvc_angles[1].get::<radian>();

                // Base thrust acts forward down the Z-axis of the vehicle body
                let thrust_base = Vector3::new(0.0, 0.0, thrust_mag);

                // TVC servos gimbal the nozzle rotating the thrust vector
                let tvc_rotation = Rotation3::from_euler_angles(tvc_pitch, tvc_yaw, 0.0);
                let active_thrust = tvc_rotation * thrust_base;

                // Calculate dynamic Center of Gravity (CoG)
                let body_length = self.config.geometry.body_length.get::<meter>();
                let dry_m = sub_state.dry_mass.value;
                let prop_m = sub_state.propellant_mass.value;
                // Assume dry mass center is at geometric z=0 (middle)
                // Assume propellant mass center is at z = -body_length / 4.0 (lower half)
                let cg_z = (0.0 * dry_m + (-body_length / 4.0) * prop_m) / (dry_m + prop_m);

                // Introduce rotational torque from off-axis thrust relative to current CoG
                // Nozzle is at the base of the rocket (-body_length/2.0 from geometric center)
                let nozzle_offset = Vector3::new(0.0, 0.0, -body_length / 2.0 - cg_z);
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
