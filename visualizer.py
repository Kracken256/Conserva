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
        scale=(10, 10, 10),
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

    # Minecraft-style free camera
    camera_controller = Entity(position=(0, 20, -50))
    camera.parent = camera_controller
    mouse.locked = True

    # Extend the camera's far clipping plane so the ground doesn't disappear at high altitudes
    camera.clip_plane_far = 1000000

    # UI Elements for telemetry display
    info_text = Text(text="Waiting for data...\nControls: [Space] Pause/Play  [R] Restart\n[Left/Right] Step  [F] Focus Rocket\n[WASD/Space/Ctrl] Fly  [Mouse] Look\n[Tab] Toggle Mouse Lock",
                     position=window.top_left, scale=1.5)

    simulation_history = []
    playback_index = 0
    is_playing = True

    def input(key):
        global is_playing, playback_index
        if key == 'space':
            is_playing = not is_playing
        elif key == 'r':
            playback_index = 0
            is_playing = True
            trail_points.clear()
            trail.model.vertices = []
            trail.model.generate()
        elif key == 'left arrow' or key == 'left arrow hold':
            is_playing = False
            playback_index = max(0, playback_index - 1)
        elif key == 'right arrow' or key == 'right arrow hold':
            is_playing = False
            playback_index = min(
                len(simulation_history) - 1, playback_index + 1)
        elif key == 'f':
            # Focus camera on the rocket
            camera_controller.position = rocket.position - camera.forward * 50
        elif key == 'escape' or key == 'tab':
            # Toggle mouse lock for Minecraft-style freecam
            mouse.locked = not mouse.locked

    def update():
        global playback_index, is_playing

        # Drain the queue to keep history updated
        while not data_queue.empty():
            simulation_history.append(data_queue.get())

        if not simulation_history:
            return

        if is_playing:
            playback_index += 1
            if playback_index >= len(simulation_history):
                playback_index = len(simulation_history) - 1
                # Only pause automatically if the simulation actually finished (rust produced no more data)
                # We can guess it's over if the thread is dead
                if not sim_thread.is_alive() and data_queue.empty():
                    is_playing = False

        state_to_render = simulation_history[playback_index]

        if 'target' in state_to_render and state_to_render['target'] is not None:
            tx, ty, tz = state_to_render['target']
            target_waypoint.position = (tx, tz, ty)

        # Minecraft style mouse look
        if mouse.locked:
            camera_controller.rotation_y += mouse.velocity[0] * 40
            camera.rotation_x -= mouse.velocity[1] * 40
            camera.rotation_x = clamp(camera.rotation_x, -90, 90)

        # Basic WASD movement that doesn't require holding right-click
        move_speed = 100 * time.dt
        if held_keys['shift']:
            move_speed *= 3

        if held_keys['w']:
            camera_controller.position += camera.forward * move_speed
        if held_keys['s']:
            camera_controller.position -= camera.forward * move_speed
        if held_keys['d']:
            camera_controller.position += camera.right * move_speed
        if held_keys['a']:
            camera_controller.position -= camera.right * move_speed
        if held_keys['space'] or held_keys['e']:
            camera_controller.position += camera.up * move_speed
        if held_keys['control'] or held_keys['q']:
            camera_controller.position -= camera.up * move_speed

        pos = state_to_render['pos']
        ux, uy, uz = pos[0], pos[2], pos[1]
        rocket.position = (ux, uy, uz)

        # --- Orientation mapping from Rust body frame to Ursina world ---
        # Rust:
        #   - Z-up world
        #   - Rocket body forward axis = +Z
        # Ursina/Panda3D:
        #   - Y-up, forward = +Z
        # We avoid hand-converting quaternion components and instead:
        #   1. Use the Rust quaternion to rotate body basis vectors into Rust world
        #   2. Remap those world vectors into Ursina coordinates
        #   3. Use lookAt(forward, up) so the visual rocket matches the sim exactly.

        i, j, k, w = state_to_render['ori']
        q_rust = Quat(w, i, j, k)

        # Body-frame basis vectors in Rust
        nose_body_rust = Vec3(0, 0, 1)  # +Z forward (rocket nose)
        up_body_rust = Vec3(0, 1, 0)    # +Y up (toward fins/"roof")

        # Rotate them into Rust world frame
        nose_world_rust = q_rust.xform(nose_body_rust)
        up_world_rust = q_rust.xform(up_body_rust)

        # Map Rust world (x, y, z) -> Ursina (x, z, y)
        def rust_to_ursina(v):
            return Vec3(v.x, v.z, v.y)

        nose_u = rust_to_ursina(nose_world_rust)
        up_u = rust_to_ursina(up_world_rust)

        # Force the visual model's +Z axis (nose) to align with the
        # physics forward vector, preserving roll via up_u.
        rocket.lookAt(rocket.position + nose_u, up_u)

        vel = state_to_render['vel']

        # Rebuild trail based on history up to current playback frame to allow scrubbing
        # We sample every 5th frame for the trail to avoid thousands of vertices if we rewind a lot
        if not is_playing:
            # If scrubbing, reconstruct the entire trail cheaply
            trail_pts = []
            for idx in range(0, playback_index + 1, 5):
                p = simulation_history[idx]['pos']
                trail_pts.append(Vec3(p[0], p[2], p[1]))
            # add the very last point
            trail_pts.append(rocket.position)
            trail.model.vertices = trail_pts
            trail.model.generate()
        else:
            # If playing forward normally, just append
            if len(trail_points) == 0 or distance(rocket.position, trail_points[-1]) > 5.0:
                trail_points.append(rocket.position)
                if len(trail_points) > 1:
                    trail.model.vertices = trail_points
                    trail.model.generate()

        status = "Live" if (is_playing and playback_index ==
                            len(simulation_history) - 1) else "REPLAY"
        status = "PAUSED" if not is_playing else status

        info_text.text = (
            f"Conserva Telemetry [{status}]\n"
            f"Frame: {playback_index}/{len(simulation_history) - 1}\n"
            f"ALT: {uy:.1f} m\n"
            f"SPD: {(vel[0]**2 + vel[1]**2 + vel[2]**2)**0.5:.1f} m/s\n"
            f"X: {pos[0]:.1f} Y: {pos[1]:.1f} Z: {pos[2]:.1f}\n"
            f"Controls: [Space] Pause/Play  [R] Restart\n"
            f"[Left/Right] Step Scrub  [F] Focus Rocket\n"
            f"[WASD / Space / Ctrl] Fly  [Mouse] Look\n"
            f"[Tab] Toggle Mouse Lock"
        )

    app.run()
