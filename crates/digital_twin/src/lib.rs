mod defaults;
mod flight_computer;
mod mesh_generation;
mod solver;

pub mod prelude {
    pub use crate::defaults::{get_default_config, get_initial_state};
    pub use crate::flight_computer::TheFlightComputer;
    pub use crate::mesh_generation::TheMeshGenerator;
    pub use crate::solver::TheSolver;
}
