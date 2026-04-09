#!/usr/bin/env python3

import subprocess
import threading
import queue
import json
import sys

try:
    from ursina import *
    from panda3d.core import Quat
except ImportError:
    print("Ursina is not installed. Please install it first:")
    print("    pip install ursina")
    sys.exit(1)

if not os.path.exists('rocket.obj'):
    print("3D model 'rocket.obj' not found. Please ensure it is in the same directory as this script.")
    sys.exit(1)


def run_simulation(data_queue):
    # Run the rust simulation and pipe stdout
    process = subprocess.Popen(
        ['cargo', 'run', '--release'],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )

    # Read output line by line as it is flushed
    for line in iter(process.stdout.readline, ''):
        line = line.strip()
        if not line:
            continue
        try:
            # Parse json state payload from the rust program
            state = json.loads(line)
            data_queue.put(state)
        except json.JSONDecodeError:
            # Print non-json logs (like compiling info or 'Target hit!' messages) to terminal
            print(f"Rust Output: {line}")

    process.stdout.close()
    process.wait()


if __name__ == '__main__':
    # Shared queue for thread-safe cross-communication
    data_queue = queue.Queue()

    # Spin up the physics runner thread natively
    sim_thread = threading.Thread(target=run_simulation, args=(data_queue,))
    sim_thread.daemon = True
    sim_thread.start()

    # --- Ursina Game Engine Setup ---
    app = Ursina()

    window.title = "Conserva 3D Flight Telemetry Tracker"
    window.borderless = False
    window.exit_button.visible = False

    # A simple checkerboard floor to act as the ground
    ground = Entity(
        model='plane',
        texture='grass',
        collider='box',
        scale=(5000, 1, 5000),
        color=color.green,
        y=-0.5
    )

    # 3D model representation of the rocket
    rocket = Entity()
    # A container for the visual mesh to pitch it forward 90 degrees so the nose aligns with the +Z (forward) axis
    # The real model mesh is provided via rocket.obj
    rocket_mesh = Entity(
        parent=rocket,
        model='rocket.obj',
        color=color.white,
        # Scaled up 15x so it's easily visible from 80 meters away
        scale=(15, 15, 15)
    )

    target_waypoint = Entity(
        model='sphere',
        color=color.red,
        scale=(50, 50, 50),
        # Ensure it maps to Ursina coordinates (z mapped to y)
        position=(0, 0, 0)
    )
    # Add a white ring to make it look like a target
    Entity(parent=target_waypoint, model='sphere',
           color=color.white, scale=(0.7, 0.7, 0.7))
    # Add an inner red ring
    Entity(parent=target_waypoint, model='sphere',
           color=color.red, scale=(0.4, 0.4, 0.4))

    # Rocket Flight Path Line trail
    trail = Entity(model=Mesh(vertices=[], mode='line',
                   thickness=2), color=color.cyan)
    trail_points = []

    # Camera controller is disabled to prevent fighting with our auto-framing script
    # and causing high-frequency vibration/ghosting.
    # camera_controller = EditorCamera()

    # Start looking down at the world from high above
    camera.position = (500, 3000, -500)
    camera.look_at((500, 0, 500))
    # Extend the camera's far clipping plane so the ground doesn't disappear at high altitudes
    camera.clip_plane_far = 1000000

    # UI Elements for telemetry display
    info_text = Text(text="Waiting for data...",
                     position=window.top_left, scale=1.5)

    def update():
        # Empty queue fully every frame so we only show the freshest state
        # (physics runs faster than graphics frame rate likely)
        latest_state = None
        while not data_queue.empty():
            latest_state = data_queue.get()

        if latest_state:
            if 'target' in latest_state and latest_state['target'] is not None:
                tx, ty, tz = latest_state['target']
                target_waypoint.position = (tx, tz, ty)

            # Map standard aerospace coordinate frames (Z is up) to Ursina's game frame (Y is up)
            # x_ursina = target_x
            # y_ursina = target_z (up)
            # z_ursina = target_y
            pos = latest_state['pos']
            ux, uy, uz = pos[0], pos[2], pos[1]

            rocket.position = (ux, uy, uz)

            # Map quaternions properly from Z-up to Y-up
            i, j, k, w = latest_state['ori']
            # nalgebra / rust produces q = w + xi + yj + zk
            # ursina uses generic quaternion representations.
            # Depending on coordinate handedness, we assign the rotation
            rocket.quaternion = Quat(w, i, j, k)

            vel = latest_state['vel']

            # Record visual trail
            if len(trail_points) == 0 or distance(rocket.position, trail_points[-1]) > 5.0:
                trail_points.append(rocket.position)
                if len(trail_points) > 1:
                    trail.model.vertices = trail_points
                    trail.model.generate()

            info_text.text = (
                f"Conserva Telemetry Live\n"
                f"ALT: {uy:.1f} m\n"
                f"SPD: {(vel[0]**2 + vel[1]**2 + vel[2]**2)**0.5:.1f} m/s\n"
                f"X: {pos[0]:.1f} Y: {pos[1]:.1f} Z: {pos[2]:.1f}"
            )

        # --- RUN EVERY RENDER FRAME (60Hz+) ---
        # Auto-framing camera heavily weighted towards the rocket to keep it close
        center_x = rocket.x * 0.8 + target_waypoint.x * 0.2
        center_y = rocket.y * 0.8 + target_waypoint.y * 0.2
        center_z = rocket.z * 0.8 + target_waypoint.z * 0.2

        # Calculate absolute distance between the two bodies
        dist = distance(rocket.position, target_waypoint.position)

        # Significantly reduced padding to make the camera much closer to the missile
        padding = max(dist * 0.4, 80.0)

        desired_pos = Vec3(center_x, center_y + padding * 0.5,
                           center_z - padding)

        camera.position = lerp(camera.position, desired_pos, time.dt * 5.0)

        # Look straight at the rocket itself rather than the empty midpoint
        camera.look_at(rocket.position)

    app.run()
