mod defaults;
mod mesh_generation;
mod rocket;

pub mod prelude {
    pub use crate::defaults::{get_default_config, get_initial_state};
    pub use crate::mesh_generation::TheMeshGenerator;
    pub use crate::rocket::TheRocket;
}
