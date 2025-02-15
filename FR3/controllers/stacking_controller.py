from typing import List

import isaacsim.robot.manipulators.controllers as manipulators_controllers
from isaacsim.core.prims import SingleArticulation
from isaacsim.robot.manipulators.grippers.parallel_gripper import ParallelGripper

from .pick_place_controller import (
    PickPlaceController,
)


class StackingController(manipulators_controllers.StackingController):
    """[summary]

    Args:
        name (str): [description]
        gripper (ParallelGripper): [description]
        robot_prim_path (str): [description]
        picking_order_cube_names (List[str]): [description]
        robot_observation_name (str): [description]
    """

    def __init__(
        self,
        name: str,
        gripper: ParallelGripper,
        robot_articulation: SingleArticulation,
        picking_order_cube_names: List[str],
        robot_observation_name: str,
    ) -> None:
        manipulators_controllers.StackingController.__init__(
            self,
            name=name,
            pick_place_controller=PickPlaceController(
                name=name + "_pick_place_controller",
                gripper=gripper,
                robot_articulation=robot_articulation,
            ),
            picking_order_cube_names=picking_order_cube_names,
            robot_observation_name=robot_observation_name,
        )
        return

    def get_current_event(self) -> int:
        _current_phase = self._pick_place_controller.get_current_event()
        _current_cube_name = self._picking_order_cube_names[self._current_cube]

        return _current_cube_name, _current_phase
