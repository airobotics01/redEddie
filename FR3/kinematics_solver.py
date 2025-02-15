from typing import Optional

import isaacsim.robot_motion.motion_generation.interface_config_loader as interface_config_loader
from isaacsim.core.prims import SingleArticulation
from isaacsim.robot_motion.motion_generation.articulation_kinematics_solver import (
    ArticulationKinematicsSolver,
)
from isaacsim.robot_motion.motion_generation.lula.kinematics import LulaKinematicsSolver


class KinematicsSolver(ArticulationKinematicsSolver):
    """Kinematics Solver for Franka robot.  This class loads a LulaKinematicsSovler object

    Args:
        robot_articulation (SingleArticulation): An initialized Articulation object representing this Franka
        end_effector_frame_name (Optional[str]): The name of the Franka end effector.  If None, an end effector link will
            be automatically selected.  Defaults to None.
    """

    def __init__(
        self,
        robot_articulation: SingleArticulation,
        end_effector_frame_name: Optional[str] = None,
    ) -> None:
        kinematics_config = interface_config_loader.load_supported_lula_kinematics_solver_config(
            "FR3"
        )  # C:\isaacsim\exts\isaacsim.robot_motion.motion_generation\motion_policy_configs
        print(kinematics_config)
        self._kinematics = LulaKinematicsSolver(**kinematics_config)

        if end_effector_frame_name is None:
            end_effector_frame_name = "fr3_rightfinger"

        ArticulationKinematicsSolver.__init__(
            self, robot_articulation, self._kinematics, end_effector_frame_name
        )

        return
