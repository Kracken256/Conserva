## Technical Specification: Aero-Structural & Control Configuration for High-Fidelity Flight Simulation

This document provides a comprehensive overview of the geometric, mass, and control parameters utilized by the simulator. The system is architected to facilitate **Six Degrees of Freedom (6-DOF)** analysis, enabling precise modeling of aerospace vehicles across various Mach regimes.

---

### I. Aerodynamic Geometry & Structural Morphology

The airframe is decomposed into three primary constituent parts: the nosecone, the cylindrical body, and the fin assembly.

#### 1. Nosecone Geometric Archetypes (`NoseconeShape`)

The nosecone dictates the initial wave drag and stagnation pressure. The simulator supports several mathematical profiles:

- **Conical:** A linear-tapered section. While simple to fabricate, it is prone to flow separation at the body-shoulder junction.
- **Ogive:** Defined by circular arcs.
  - _Tangent Ogive:_ The arc is tangent to the body at the shoulder, minimizing pressure discontinuities.
  - _Secant Ogive:_ Allows for a more "aggressive" curve where the arc is not tangent, often utilized to optimize internal volume.
- **Haack Series:** Analytical shapes derived from the **Seas-Haack** equations to minimize wave drag for a given length and diameter.
  - _LD-Haack (Von Kármán):_ Optimized for minimum drag.
  - _LV-Haack:_ Optimized for minimum drag given a specific internal volume.
- **Power Series & Parabolic:** Specialized curves for hypersonic thermal management. The `blunting_radius` parameter is critical here for simulating "blunt-body" effects to mitigate aerodynamic heating at high Mach numbers.

#### 2. Fin Planform & Edge Profiling (`FinGeometry`)

Fins are modeled as symmetric lifting surfaces. The simulator captures complex planforms through parametric chord and span definitions:

- **Planform Parameters:** Root chord, tip chord, span, and sweep length are used to compute the **Center of Pressure (CP)** and surface area.
- **Edge Contouring:** The `FinEdgeProfile` allows for `Curved` leading and trailing edges. A convex profile (positive depth) increases lift-curve slope, while concave profiles (negative depth) can be used for specialized stability tuning.
- **Chamfering:** Structural tapering via `edge_chamfer` is essential for realistic profile drag calculations, transitioning the fin from a blunt "plate" to an aerodynamic "foil."

---

### II. Dynamic Mass & Inertial Properties (`MissileMassConfig`)

The simulator treats the vehicle as a variable-mass system. As propellant is expelled, the following properties are updated via time-series interpolation:

| Property           | Definition                                                   | Simulation Impact                                                            |
| :----------------- | :----------------------------------------------------------- | :--------------------------------------------------------------------------- |
| **Mass Migration** | Transition from `wet_mass` to `dry_mass`.                    | Modifies acceleration ($F=ma$) and gravitational pull.                       |
| **CG Drift**       | The `cg_curve` tracks the 3D Center of Gravity.              | Changes the static margin (distance between CG and CP), affecting stability. |
| **Inertia Tensor** | A time-variant $3 \times 3$ matrix (`inertia_tensor_curve`). | Dictates resistance to rotational moments (pitch, yaw, roll).                |

---

### III. Guidance, Navigation, and Control (GNC)

The control architecture is divided between the software logic (PID) and the physical hardware constraints (Engine/TVC).

#### 1. PID Control Loops (`MissileControllerConfig`)

The simulator utilizes parallel PID controllers for the pitch and yaw axes:

- **Proportional ($K_p$):** Immediate reaction to orientation error.
- **Integral ($K_i$):** Eliminates steady-state error (e.g., gravity-induced "sag" during long-range flight).
- **Derivative ($K_d$):** Provides damping to prevent "fishtailing" and overshoots during high-speed maneuvers.

#### 2. Actuator & Engine Dynamics (`MissileEngineConfig`)

To ensure the simulation remains grounded in physical reality, the Thrust Vector Control (TVC) system includes:

- **Mechanical Hard-Stops:** `max_tvc_angle` prevents the controller from commanding physically impossible gimbal deflections.
- **Slew Rate Limits:** The `tvc_slew_rate` models the maximum speed of the gimbal servos.
- **Electromechanical Latency:** The `tvc_activation_delay` (time constant $\tau$) simulates the first-order lag inherent in any physical servo-mechanical system.

---

### IV. Simulator Limitations & Assumptions

While high-fidelity, the current geometric engine operates under the following constraints:

1.  **Radial Symmetry:** All fins in a `fin_set` are assumed to be identical and spaced evenly ($360^\circ / \text{num\_fins}$).
2.  **Rigid-Body Dynamics:** The airframe and fins are assumed to be perfectly rigid. **Aeroelasticity** (vibration, flutter, or structural bending) is not currently modeled.
3.  **Constant Body Diameter:** The `cylindrical_body_length` assumes a uniform diameter. The engine does not natively support "boattails" (tapered aft sections) or "flares" without manual mesh intervention.
4.  **Single-Stage Focus:** The current configuration is optimized for single-stage vehicles; multi-stage separation events require external state-machine handling.
