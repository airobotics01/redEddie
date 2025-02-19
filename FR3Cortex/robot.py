from abc import abstractmethod
from collections import OrderedDict
from typing import Dict, Optional, Sequence

import isaacsim.robot_motion.motion_generation.interface_config_loader as icl
import numpy as np
import omni.physics.tensors
from isaacsim.core.api.articulations import ArticulationSubset
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.cortex.framework.commander import Commander
from isaacsim.cortex.framework.cortex_utils import get_assets_root_path_or_die
from isaacsim.cortex.framework.cortex_world import CommandableArticulation, CortexWorld
from isaacsim.cortex.framework.motion_commander import (
    CortexObstacleType,
    MotionCommander,
)
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper
from isaacsim.robot_motion.motion_generation.articulation_motion_policy import (
    ArticulationMotionPolicy,
)
from isaacsim.robot_motion.motion_generation.lula.motion_policies import (
    RmpFlow,
    RmpFlowSmoothed,
)


from isaacsim.cortex.framework.robot import MotionCommandedRobot
from isaacsim.cortex.framework.robot import CortexGripper


class FR3Gripper(CortexGripper):
    """Franka specific parallel gripper.

    Specifies the gripper joints, provides mappings from width to joints, and defines the franka
    opened and closed widths.

    Args:
        articulation: The Articulation object containing the finger joints that will be controlled
            by this parallel graipper.
    """

    def __init__(self, articulation: SingleArticulation):
        super().__init__(
            articulation_subset=ArticulationSubset(
                articulation, ["fr3_finger_joint1", "fr3_finger_joint2"]
            ),
            opened_width=0.08,
            closed_width=0.0,
        )

    def joints_to_width(self, joint_positions: Sequence[float]) -> float:
        """The width is simply the sum of the two prismatic joints.

        Args:
            joint_positions: The values for joints ["fr3_finger_joint1", "fr3_finger_joint2"].

        Returns:
            The width of the gripper corresponding to those joint positions.
        """
        return joint_positions[0] + joint_positions[1]

    def width_to_joints(self, width: float) -> np.ndarray:
        """Each joint is half of the width since the width is their sum.

        Args:
            width: The width of the gripper

        Returns:
            The values for joints ["fr3_finger_joint1", "fr3_finger_joint2"] giving the
            requested gripper width.
        """
        return np.array([width / 2, width / 2])


class CortexFR3(MotionCommandedRobot):
    """The Cortex Franka contains commanders for commanding the end-effector (a MotionCommander
    governing the full arm) and the gripper (a FR3Gripper governing the fingers).

    Each of these commanders are accessible via members arm and gripper.

    This object only wraps an existing USD Franka on the stage at the specified prim_path. To
    add it to the stage first then wrap it, use the add_franka_to_stage() method.

    Note that position and orientation are both relative to the prim the Franka sits on.

    Args:
        name: A name for the Franka robot. Robots added to the CortexWorld should all have unique names.
        prim_path: The path to the Franka prim in the USD stage.
        position: The position of the robot. See CortexRobot's position parameter for details.
        orientation: The orientation of the robot. See CortexRobot's orientation parameter for details.
        use_motion_commander: When set to True (default), uses the motion commander. Otherwise,
            includes only a DirectSubsetCommander for the arm.
    """

    def __init__(
        self,
        name: str,
        prim_path: str,
        position: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
        use_motion_commander=True,
    ):
        motion_policy_config = icl.load_supported_motion_policy_config("FR3", "RMPflow")

        motion_policy_config["maximum_substep_size"] = 0.00167  # 시뮬 정확도 높이기?
        motion_policy_config["ignore_robot_state_updates"] = True  # 안정성? 특정조건?

        super().__init__(
            name=name,
            prim_path=prim_path,
            motion_policy_config=motion_policy_config,
            position=position,
            orientation=orientation,
            settings=MotionCommandedRobot.Settings(
                active_commander=use_motion_commander,
                smoothed_rmpflow=True,
                smoothed_commands=True,
            ),
        )

        self.gripper_commander = FR3Gripper(self)
        self.add_commander("gripper", self.gripper_commander)

    def initialize(
        self, physics_sim_view: omni.physics.tensors.SimulationView = None
    ) -> None:
        """Initializes using MotionCommandedRobot's initialize() and also adds custom setting of the
        gains.

        Users generally don't need to call this method explicitly. It's handled automatically on
        reset() when this robot is added to the CortexWorld.

        Args:
            physics_sim_view: Sim information required by the underlying Articulation initialization.
        """
        super().initialize(physics_sim_view)

        verbose = True
        kps = np.array(
            [
                6000000.0,
                6000000.0,
                6000000.0,
                6000000.0,
                2500000.0,
                1500000.0,
                500000.0,
                6000.0,
                6000.0,
            ]
        )
        kds = np.array(
            [
                300000.0,
                300000.0,
                300000.0,
                300000.0,
                90000.0,
                90000.0,
                90000.0,
                1000.0,
                1000.0,
            ]
        )
        if verbose:
            print("setting franka gains:")
            print("- kps: {}".format(kps))
            print("- kds: {}".format(kds))
        self.get_articulation_controller().set_gains(kps, kds)


def add_fr3_to_stage(
    name: str,
    prim_path: str,
    usd_path: Optional[str] = None,
    position: Optional[Sequence[float]] = None,
    orientation: Optional[Sequence[float]] = None,
    use_motion_commander=True,
):
    """Adds a Franka to the stage at the specified prim_path, then wrap it as a CortexFR3 object.

    Args:
        For name, prim_path, position, orientation, and motion_commander, see the CortexFR3 doc
        string.

        usd_path: An optional path to the Franka USD asset to add. If a specific path is not
            provided, a default Franka USD path is used.

    Returns: The constructed CortexFR3 object.
    """
    if usd_path is not None:
        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
    else:
        usd_path = get_assets_root_path_or_die() + "/Isaac/Robots/Franka/FR3/fr3.usd"
        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

    return CortexFR3(name, prim_path, position, orientation, use_motion_commander)
