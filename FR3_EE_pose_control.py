from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})


import numpy as np
import os
import carb

from omni.isaac.core import World
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import Articulation
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats
from omni.isaac.motion_generation import (
    ArticulationKinematicsSolver,
    LulaKinematicsSolver,
)

# Create a World
my_world = World(stage_units_in_meters=1.0)

# Add a ground plane
my_world.scene.add_default_ground_plane()

# Add the FR3 robot to the stage
robot_prim_path = "/World/FR3"
path_to_robot_usd = get_assets_root_path() + "/Isaac/Robots/Franka/FR3/fr3.usd"
add_reference_to_stage(path_to_robot_usd, robot_prim_path)
articulation = Articulation(robot_prim_path)
my_world.scene.add(articulation)

# Add the target to the stage
add_reference_to_stage(
    get_assets_root_path() + "/Isaac/Props/UIElements/frame_prim.usd", "/World/target"
)
target = XFormPrim("/World/target", scale=[0.04, 0.04, 0.04])
target.set_default_state(np.array([0.3, 0, 0.5]), euler_angles_to_quats([0, np.pi, 0]))
my_world.scene.add(target)

# Setup the kinematics solver
mg_extension_path = get_extension_path_from_name("omni.isaac.motion_generation")
kinematics_config_dir = os.path.join(mg_extension_path, "motion_policy_configs")

kinematics_solver = LulaKinematicsSolver(
    robot_description_path=kinematics_config_dir
    + "/FR3/rmpflow/fr3_robot_description.yaml",
    urdf_path=kinematics_config_dir + "/FR3/fr3.urdf",
)

print(
    "Valid frame names at which to compute kinematics:",
    kinematics_solver.get_all_frame_names(),
)

end_effector_name = "fr3_hand"
articulation_kinematics_solver = ArticulationKinematicsSolver(
    articulation, kinematics_solver, end_effector_name
)

# Reset the world
my_world.reset()

# Run the simulation
while simulation_app.is_running():
    my_world.step(render=True)

    target_position, target_orientation = target.get_world_pose()
    robot_base_translation, robot_base_orientation = articulation.get_world_pose()
    kinematics_solver.set_robot_base_pose(
        robot_base_translation, robot_base_orientation
    )

    action, success = articulation_kinematics_solver.compute_inverse_kinematics(
        target_position, target_orientation
    )

    if success:
        articulation.apply_action(action)
    else:
        carb.log_warn("IK did not converge to a solution. No action is being taken")

    if my_world.current_time_step_index >= 1000:  # Run for 1000 steps
        break

# Cleanup
simulation_app.close()
