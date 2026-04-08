use crate::control::flight_computer::FlightComputer;
use crate::control::state::MissileState;
use crate::geometry::config::MissileConfig;
use crate::geometry::mesh::{Mesh, MeshGenerator};
use crate::integ;
use crate::physics::solver::AeroSolver;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use uom::si::angular_velocity::{AngularVelocity, radian_per_second};
use uom::si::f32::{Length, Velocity};
use uom::si::length::meter;
use uom::si::velocity::meter_per_second;

pub struct DigitalTwin {
    pub config: MissileConfig,
    pub state: MissileState,
    pub flight_computer: Box<dyn FlightComputer>,
    pub solver: Box<dyn AeroSolver>,
    pub mesh_generator: Box<dyn MeshGenerator>,
    pub current_mesh: Mesh,
    rk4: integ::rk4::Rk4,
}

impl DigitalTwin {
    pub fn new(
        config: MissileConfig,
        state: MissileState,
        solver: Box<dyn AeroSolver>,
        mesh_generator: Box<dyn MeshGenerator>,
        flight_computer: Box<dyn FlightComputer>,
    ) -> Self {
        Self {
            config,
            state,
            solver,
            mesh_generator,
            flight_computer,
            current_mesh: Mesh::default(),
            rk4: integ::rk4::Rk4 {},
        }
    }

    pub fn step(&mut self, dt: f32) {
        // 1. Update Flight Computer first (to get new fin angles/thrust settings)
        // The FC sets the "intent" for the next step.
        self.state = self.flight_computer.update(&self.state, dt);

        // 2. Define the "System Dynamics"
        let physics_engine =
            |v: &Vector3<f32>, w: &Vector3<f32>, q: &Quaternion<f32>, p: &Vector3<f32>| {
                // Create a temporary state for the sub-step
                let mut sub_state = self.state.clone();

                // Map raw f32s back into uom quantities
                sub_state.position = p.map(Length::new::<meter>);
                sub_state.body_velocity = v.map(Velocity::new::<meter_per_second>);
                sub_state.angular_velocity = w.map(AngularVelocity::new::<radian_per_second>);
                sub_state.orientation = UnitQuaternion::from_quaternion(*q);

                // Update mesh for this sub-step (now with altitude/position awareness!)
                self.mesh_generator
                    .generate(&sub_state, &self.config, &mut self.current_mesh);

                let output = self.solver.calculate_forces(&self.current_mesh, &sub_state);
                (output.force, output.torque)
            };

        // 3. Let RK4 run the show
        self.state = self.rk4.step(&self.state, physics_engine, dt);
    }
}
