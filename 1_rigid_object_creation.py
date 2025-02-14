import os
from isaacsim.simulation_app import SimulationApp

os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES"
CONFIG = {
    "renderer": "RayTracedLighting",
    "headless": False,
    "width": 1280,
    "height": 720,
    "num_frames": 30,
}
simulation_app = SimulationApp(CONFIG)


import numpy as np
from isaacsim.core.api.world import World
from isaacsim.core.api.objects import DynamicCuboid, DynamicCylinder, DynamicSphere
from omni.kit.viewport.utility import get_active_viewport

# Create a world and add the default ground plane
my_world = World()
my_world.scene.add_default_ground_plane()

# Create a dynamic cube (cuboid)
dynamic_cuboid = DynamicCuboid(
    prim_path="/World/Cube",
    name="my_cube",
    position=[1.0, 0.5, 1.0],
    orientation=[1, 0, 0, 0],
    color=np.array([0.2, 0.3, 0.0]),
    size=0.5,
    scale=[0.5, 0.5, 0.5],
)
my_world.scene.add(dynamic_cuboid)

# Create a dynamic cylinder
dynamic_cylinder = DynamicCylinder(
    prim_path="/World/Cylinder",
    name="my_cylinder",
    position=[-1.0, 0.5, 1.0],
    orientation=[0.7071, 0, 0.7071, 0],
    color=np.array([0.1, 0.4, 0.3]),
    radius=0.25,
    height=0.8,
    linear_velocity=np.array([0, 1, 0]),
)
my_world.scene.add(dynamic_cylinder)

# Create a dynamic sphere
dynamic_sphere = DynamicSphere(
    prim_path="/World/Sphere",
    name="my_sphere",
    position=[0.0, 0.5, 1.0],
    orientation=[1, 0, 0, 0],
    color=np.array([0.5, 0.2, 0.7]),
    radius=0.3,
    linear_velocity=np.array([0, -1, 0]),
)
my_world.scene.add(dynamic_sphere)

# Setup viewport information
viewport = get_active_viewport()
resolution = viewport.get_texture_resolution()  # Returns (width, height)
print("Current viewport resolution:", resolution)

my_world.reset()

# Main simulation loop
previous_state = None
while simulation_app.is_running():
    my_world.step(render=True)

    current_state = f"Cube position: {dynamic_cuboid.get_world_pose()[0]}"
    if previous_state != current_state:
        print(current_state)
        previous_state = current_state

simulation_app.close()
