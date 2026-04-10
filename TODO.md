# Conserva - Future Roadmap & TODOs

## Aerodynamics & Geometry
- [ ] **Boattails & Transitions**: Add support for non-constant body diameters in `MissileGeometryConfig` to model tapered aft sections and flares, improving base drag calculations.
- [ ] **Canards & Grid Fins**: Implement forward-mounted control surfaces and complex grid fin geometries for simulating different vehicle archetypes.
- [ ] **Roll Control**: Incorporate fin cant angles or active independent fin actuation to unlock 3-axis control (Pitch, Yaw, Roll) alongside TVC.

## Guidance, Navigation, and Control (GNC)
- [ ] **Proportional Navigation (PN)**: Implement PN or Augmented Proportional Navigation (APN) guidance laws to intercept moving targets rather than static waypoints.
- [ ] **Sensor Noise & Estimation**: Simulate noisy IMUs (gyro drift, accelerometer bias) and implement an Extended Kalman Filter (EKF) to estimate the true vehicle state.

## Environmental Physics
- [ ] **Wind & Turbulence Modeling**: Introduce atmospheric disturbances such as steady crosswinds and randomized turbulence gusts (e.g., Dryden model).
- [ ] **Advanced Atmosphere (ISA)**: Ensure the atmosphere model dynamically adjusts temperature, pressure, density, and local speed of sound against extreme altitude variations.

## Architecture & Tooling
- [ ] **Native Python Bindings (PyO3)**: Rewrite the IPC boundary using the `pyo3` crate to compile Conserva as a native Python extension, eliminating stdout serialization overhead.
- [ ] **Flight Data Recorder**: Implement telemetry export to high-performance formats (HDF5, Parquet, or CSV) for post-flight analysis in Jupyter/Matplotlib or playback in the visualizer.
