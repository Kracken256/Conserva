# Conserva

Conserva is a high-fidelity Digital Twin and 6-DOF (Six Degrees of Freedom) physics simulation engine designed specifically for guided rockets and missiles. It provides comprehensive aerodynamic modeling, continuous state-space tracking, and procedural 3D vehicle generation.

## Features

- **Advanced Aerodynamic Simulation**: Handles flow characteristics across subsonic, transonic, supersonic, and hypersonic regimes using dynamic mass and shifting inertia tensors.
- **Procedural 3D Mesh Generation**: Programmatically outputs exact 3D models of the vehicle based on the aerodynamic configurations. Features full support for standard nosecone profiles (Von Karman, Ogive, Parabolic, Elliptical) and highly complex swept fin geometries with chamfered and curved edge profiles.
- **Precision Guidance & Control**: Simulates flight control hardware and software, integrating multi-axis PID loops against physical Thrust Vector Control (TVC) systems. Models real-world limitations including TVC slew rates, activation delays, and maximum deflection angles.
- **Dynamic Mass Handling**: Tracks variable thrust profiles, moving centers of gravity (CG), and deteriorating inertia matrices natively over the duration of solid motor propellant burns.

## Project Structure

The workspace is organized into a main simulation binary wrapped by a Python visualization layer, supported by several modular Rust crates:

- **`src/main.rs` (Conserva Root Binary)**: The primary entry point. It instantiates the default `MissileConfig`, steps the 6-DOF RK4 physics engine loop, and streams telemetry JSON data to standard out.
- **`visualizer.py`**: A native 3D visualization and dashboard tool built with the Ursina game engine. It executes the Conserva binary in a background thread and parses the stdout JSON stream in real-time.
- **`crates/digital_twin`**: The concrete implementations for the simulation. Contains the procedural 3D vehicle mesh generator (`TheMeshGenerator`), the active guidance loop logic (`TheRocket`), and the initial default configurations.
- **`crates/digital_twin_glue`**: The mathematical framework bridging the components. It houses the 6-DOF physics solver, RK4 integrators, aerodynamic force calculators, and the overarching `DigitalTwin` execution manager.
- **`crates/mesh_export`**: A dedicated utility utility that generates and exports the dynamic procedural 3D vehicle mesh directly to a `.obj` format file for external CFD or graphic analysis.
- **`crates/cp_export`**: A utility program engineered to evaluate aerodynamic characteristics and sweep the varying Center of Pressure (CP) values to a log over different flow regimes.
- **`crates/tuner`**: An optimization tool executing a metaheuristic Grey Wolf Optimizer (GWO) across the Rayon thread pool to automatically tune dynamic flight PID controller constants.

## Building and Running

Conserva relies natively on Rust and a lightweight Python 3 environment.

To test the integration and math invariants:

```bash
cargo test --workspace
```

To run the 3D Ursina Visualization Engine locally:

1. Build all dependencies and prepare the visualization model:

   ```bash
   cargo run --release -p mesh_export
   ```

   _This procedural pass builds your unique `rocket.obj` into the active directory based on your default aerodynamic configuration._

2. Install the necessary Python packages within a virtual environment:

   ```bash
   pip install ursina panda3d
   ```

3. Launch the visualizer wrapper:
   ```bash
   ./visualizer.py
   ```

## Configuration

The primary input for the digital twin is the `MissileConfig` schema, which orchestrates four primary modules:

1. `Geometry`: Bounds dimensions and form factors (body, nosecone, fin constraints).
2. `Controller`: Holds PID values handling active trajectory deflections.
3. `Engine`: Contains impulse mapping and mechanical TVC servo limits.
4. `Mass`: Tracks standard wet/dry weights alongside temporal mass & inertia curves.
