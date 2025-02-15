from typing import Optional

import isaacsim.core.api.tasks as tasks
import numpy as np
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.string import find_unique_string_name

from franka import FR3


class FollowTarget(tasks.FollowTarget):
    """[summary]

    Args:
        name (str, optional): [description]. Defaults to "fr3_follow_target".
        target_prim_path (Optional[str], optional): [description]. Defaults to None.
        target_name (Optional[str], optional): [description]. Defaults to None.
        target_position (Optional[np.ndarray], optional): [description]. Defaults to None.
        target_orientation (Optional[np.ndarray], optional): [description]. Defaults to None.
        offset (Optional[np.ndarray], optional): [description]. Defaults to None.
        franka_prim_path (Optional[str], optional): [description]. Defaults to None.
        franka_robot_name (Optional[str], optional): [description]. Defaults to None.
    """

    def __init__(
        self,
        name: str = "fr3_follow_target",
        target_prim_path: Optional[str] = None,
        target_name: Optional[str] = None,
        target_position: Optional[np.ndarray] = None,
        target_orientation: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
        franka_prim_path: Optional[str] = None,
        franka_robot_name: Optional[str] = None,
    ) -> None:
        tasks.FollowTarget.__init__(
            self,
            name=name,
            target_prim_path=target_prim_path,
            target_name=target_name,
            target_position=target_position,
            target_orientation=target_orientation,
            offset=offset,
        )
        self._franka_prim_path = franka_prim_path
        self._franka_robot_name = franka_robot_name
        return

    def set_robot(self) -> FR3:
        """[summary]

        Returns:
            FR3: [description]
        """
        if self._franka_prim_path is None:
            self._franka_prim_path = find_unique_string_name(
                initial_name="/World/FR3",
                is_unique_fn=lambda x: not is_prim_path_valid(x),
            )
        if self._franka_robot_name is None:
            self._franka_robot_name = find_unique_string_name(
                initial_name="my_fr3",
                is_unique_fn=lambda x: not self.scene.object_exists(x),
            )
        return FR3(prim_path=self._franka_prim_path, name=self._franka_robot_name)
