mod digital_twin;
mod integ;
mod physics;
mod rocket;

pub mod prelude {
    pub use crate::digital_twin::DigitalTwin;
    pub use crate::physics::{SolverOutput, calculate_forces, lookup_atmosphere};
    pub use crate::rocket::configuration::{
        EnvironmentalConfig, FinEdgeProfile, FinGeometry, MissileConfig, MissileControllerConfig,
        MissileEngineConfig, MissileGeometryConfig, MissileMassConfig, NoseconeShape,
    };
    pub use crate::rocket::mesh::{Mesh, MeshGenerator};
    pub use crate::rocket::state::{MissileState, RocketCtrl};
}
