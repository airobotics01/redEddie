from typing import Optional

import isaacsim.core.api.tasks as tasks
import numpy as np
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.string import find_unique_string_name

from franka import FR3


class PickPlace(tasks.PickPlace):
    """[summary]

    Args:
        name (str, optional): [description]. Defaults to "fr3_pick_place".
        cube_initial_position (Optional[np.ndarray], optional): [description]. Defaults to None.
        cube_initial_orientation (Optional[np.ndarray], optional): [description]. Defaults to None.
        target_position (Optional[np.ndarray], optional): [description]. Defaults to None.
        cube_size (Optional[np.ndarray], optional): [description]. Defaults to None.
        offset (Optional[np.ndarray], optional): [description]. Defaults to None.
    """

    def __init__(
        self,
        name: str = "fr3_pick_place",
        cube_initial_position: Optional[np.ndarray] = None,
        cube_initial_orientation: Optional[np.ndarray] = None,
        target_position: Optional[np.ndarray] = None,
        cube_size: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        tasks.PickPlace.__init__(
            self,
            name=name,
            cube_initial_position=cube_initial_position,
            cube_initial_orientation=cube_initial_orientation,
            target_position=target_position,
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
        gripper_closed_position = self._cube_size[1] / 2
        deltas = np.array([0.04, 0.04]) - gripper_closed_position
        return FR3(
            prim_path=franka_prim_path,
            name=franka_robot_name,
            gripper_closed_position=gripper_closed_position,
            deltas=deltas,
        )
