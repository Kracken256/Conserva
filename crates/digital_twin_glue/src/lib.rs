mod control;
mod digital_twin;
mod geometry;
mod integ;
mod physics;

pub mod prelude {
    pub use crate::control::rocket::Rocket;
    pub use crate::control::state::MissileState;
    pub use crate::digital_twin::DigitalTwin;
    pub use crate::geometry::config::MissileConfig;
    pub use crate::geometry::mesh::{Mesh, MeshGenerator};
    pub use crate::physics::solver::{AeroSolver, SolverOutput};
}
