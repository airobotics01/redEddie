from typing import Optional

import numpy as np
from isaacsim.core.api.tasks import Stacking as BaseStacking
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.string import find_unique_string_name

from franka import FR3


class Stacking(BaseStacking):
    """[summary]

    Args:
        name (str, optional): [description]. Defaults to "fr3_stacking".
        target_position (Optional[np.ndarray], optional): [description]. Defaults to None.
        cube_size (Optional[np.ndarray], optional): [description]. Defaults to None.
        offset (Optional[np.ndarray], optional): [description]. Defaults to None.
    """

    def __init__(
        self,
        name: str = "fr3_stacking",
        target_position: Optional[np.ndarray] = None,
        cube_size: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        if target_position is None:
            target_position = np.array([0.5, 0.5, 0]) / get_stage_units()
        BaseStacking.__init__(
            self,
            name=name,
            cube_initial_positions=np.array([[0.3, 0.3, 0.3], [0.3, -0.3, 0.3]])
            / get_stage_units(),
            cube_initial_orientations=None,
            stack_target_position=target_position,
            cube_size=cube_size,
            offset=offset,
        )
        return

    def set_robot(self) -> FR3:
        """[summary]

        Returns:
            FR3: [description]
        """
        franka_prim_path = find_unique_string_name(
            initial_name="/World/FR3",
            is_unique_fn=lambda x: not is_prim_path_valid(x),
        )
        franka_robot_name = find_unique_string_name(
            initial_name="my_fr3",
            is_unique_fn=lambda x: not self.scene.object_exists(x),
        )
        return FR3(prim_path=franka_prim_path, name=franka_robot_name)
