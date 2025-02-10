from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
from omni.isaac.core import World
from omni.isaac.manipulators.controllers import (
    PickPlaceController as BasePickPlaceController,
)
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.core.articulations import Articulation
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.isaac.core.tasks as tasks
import os
import numpy as np
from omni.isaac.motion_generation import RmpFlow, ArticulationMotionPolicy
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats
from scipy.spatial.transform import Rotation


class FR3RMPFlowController:
    def __init__(self, articulation, end_effector_frame_name="fr3_hand"):
        self._articulation = articulation

        # Get path to motion generation extension and FR3 configs
        mg_extension_path = get_extension_path_from_name("omni.isaac.motion_generation")
        rmp_config_dir = os.path.join(
            mg_extension_path, "motion_policy_configs", "FR3", "rmpflow"
        )

        # Initialize RMPFlow with FR3-specific configuration files
        self._rmpflow = RmpFlow(
            robot_description_path=os.path.join(
                rmp_config_dir, "fr3_robot_description.yaml"
            ),
            urdf_path=os.path.join(
                mg_extension_path, "motion_policy_configs", "FR3", "fr3.urdf"
            ),
            rmpflow_config_path=os.path.join(rmp_config_dir, "fr3_rmpflow_config.yaml"),
            end_effector_frame_name=end_effector_frame_name,
            maximum_substep_size=0.00334,
        )

        # Wrap RMPFlow with ArticulationMotionPolicy to connect it to the FR3 articulation
        self._motion_policy = ArticulationMotionPolicy(
            self._articulation, self._rmpflow
        )

    def set_end_effector_target(self, position, orientation):
        """
        Set the target position and orientation for the end effector.

        Args:
            position: Target position as a numpy array [x, y, z].
            orientation: Target orientation as a quaternion [x, y, z, w].
        """
        self._rmpflow.set_end_effector_target(position, orientation)

    def get_next_action(self, step):
        """
        Get the next action for the robot based on the current target.

        Args:
            step: The time step for the simulation.

        Returns:
            An ArticulationAction object containing joint positions and velocities.
        """
        return self._motion_policy.get_next_articulation_action(step)

    def reset(self):
        """Reset the controller."""
        self._rmpflow.reset()


# Initialize World
my_world = World(stage_units_in_meters=1.0)

# Load FR3 Robot into the Stage
robot_prim_path = "/World/FR3"
path_to_robot_usd = (
    get_assets_root_path() + "/Isaac/Robots/Franka/FR3/fr3.usd"
)  # Replace <nucleus_server_path> with your Nucleus path
add_reference_to_stage(path_to_robot_usd, robot_prim_path)

# Wrap FR3 Robot as an Articulation Object
fr3_articulation = Articulation(robot_prim_path)
my_world.scene.add(fr3_articulation)

# Initialize Custom RMPFlow Controller for FR3
controller = FR3RMPFlowController(articulation=fr3_articulation)

# Set Initial Target Position and Orientation for End Effector
target_position = np.array([0.5, 0.0, 0.7])  # Example position in world coordinates
target_orientation = euler_angles_to_quats(
    [0, np.pi / 2, 0]
)  # Example quaternion (90 degrees around Y-axis)
controller.set_end_effector_target(target_position, target_orientation)

# Reset World and Controller Before Starting Simulation
my_world.scene.add_default_ground_plane()
my_world.reset()
controller.reset()

# Main Simulation Loop
reset_needed = False
i = 0
while simulation_app.is_running():
    i += 1
    my_world.step(render=True)

    if my_world.is_stopped() and not reset_needed:
        reset_needed = True

    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            controller.reset()
            reset_needed = False

        # Compute Next Action Using RMPFlow Controller
        step_time = 1 / 60.0  # Example timestep (60 Hz)
        action = controller.get_next_action(step_time)

        # Apply Action to FR3 Robot Articulation
        fr3_articulation.apply_action(action)

        if i < 100:
            controller.set_end_effector_target(
                np.array([0.5, 0, 0.6]),
                Rotation.from_euler("xyz", [0, np.pi, 0]).as_quat(),
            )
        if i >= 100:
            controller.set_end_effector_target(
                np.array([0, 0.5, 0.6]),
                Rotation.from_euler("xyz", [0, np.pi, 0]).as_quat(),
            )
        if i >= 300:
            i = 0

simulation_app.close()
