mod mesh_generation;
mod parameters;
mod rocket_ctrl;

pub mod prelude {
    pub use crate::mesh_generation::RocketMesh;
    pub use crate::parameters::{get_rocket_design, get_rocket_initial_state};
    pub use crate::rocket_ctrl::TheRocket;
}
