import os
os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES"

CONFIG = {"renderer": "RayTracedLighting", "headless": False, 
          "width": 1980, "height": 1080, "num_frames": 24}
    
# Initialize app FIRST
from isaacsim import SimulationApp
simulation_app = SimulationApp(CONFIG)


import numpy as np
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

# Create physics context and world
world = World()
world.scene.add_default_ground_plane()

# Create rigid object
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        position=[0, 0, 1.0],
        scale=[0.5, 0.5, 0.5],
        color=np.array([.2,.3,0.]),
    )
)

from omni.kit.viewport.utility import get_active_viewport
viewport = get_active_viewport()
resolution = viewport.get_texture_resolution()  # Returns (width, height)
print("Current viewport resolution:", resolution)


# Initialize physics
world.reset()

# Main simulation loop
while simulation_app.is_running():
    # Step physics + rendering together
    world.step(render=True)
    
    # (Optional) Access object properties
    print("Cube position:", cube.get_world_pose()[0])

# Cleanup
simulation_app.close()
