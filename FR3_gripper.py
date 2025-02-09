from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
import os
import carb
from omni.isaac.core import World
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage

from omni.isaac.core import World
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.types import ArticulationAction


my_world = World(stage_units_in_meters=1.0)

robot_prim_path = "/World/FR3"
path_to_robot_usd = get_assets_root_path() + "/Isaac/Robots/Franka/FR3/fr3.usd"
add_reference_to_stage(path_to_robot_usd, robot_prim_path)

gripper = (
    ParallelGripper(  # For the finger only the first value is used, second is ignored
        end_effector_prim_path="/World/FR3/fr3_hand",
        joint_prim_names=["fr3_finger_joint1", "fr3_finger_joint2"],
        joint_opened_positions=np.array([0.04, 0.04]),
        joint_closed_positions=np.array([0, 0]),
        action_deltas=np.array([0.04, 0.04]),
    )
)

# define the manipulator
fr3_robot = SingleManipulator(
    prim_path=robot_prim_path,
    name="my_fr3",
    end_effector_prim_name="fr3_hand",
    gripper=gripper,
)
my_world.scene.add(fr3_robot)

joints_default_positions = np.array(
    [0.0, -0.3, 0.0, -1.8, 0.0, 1.5, 0.7, +0.04, -0.04]
)  # 7 arm joints and 2 finger joints
fr3_robot.set_joints_default_state(positions=joints_default_positions)

my_world.scene.add_default_ground_plane()
my_world.reset()

i = 0
reset_needed = False
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            reset_needed = False

        i += 1
        gripper_positions = fr3_robot.gripper.get_joint_positions()
        print(gripper_positions)

        if i < 150:
            # open the gripper slowly
            print("open")
            fr3_robot.gripper.apply_action(
                ArticulationAction(
                    joint_positions=[
                        gripper_positions[0] + 0.01,
                        gripper_positions[1] - 0.01,  # second value is ignored
                    ]
                )
            )

        if i > 150:
            # close the gripper slowly
            print("close")
            fr3_robot.gripper.apply_action(
                ArticulationAction(
                    joint_positions=[
                        gripper_positions[0] - 0.01,
                        gripper_positions[1] + 0.01,  # second value is ignored
                    ]
                )
            )

        if i == 300:
            i = 0
