from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
from typing import List, Optional
from isaacsim.core.api import World
from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.utils.string import find_unique_string_name
from isaacsim.core.api.objects import DynamicCuboid, DynamicCylinder
from isaacsim.core.utils.prims import is_prim_path_valid

from controllers.pick_place_controller import PickPlaceController
from tasks.pick_place import PickPlace


class PickPlaceCylinder(PickPlace):
    def __init__(
        self,
        name: str = "pick_place_cylinder",
        cylinder_initial_position: Optional[np.ndarray] = None,
        cylinder_initial_orientation: Optional[np.ndarray] = None,
        target_position: Optional[np.ndarray] = None,
        cylinder_size: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ):
        if cylinder_size is None:
            cylinder_size = np.array(
                [0.06, 0.06, 0.12]
            )  # np.array([지름, 지름(무효), 높이])
        super().__init__(
            name=name,
            cube_initial_position=cylinder_initial_position,
            cube_initial_orientation=cylinder_initial_orientation,
            target_position=target_position,
            cube_size=cylinder_size,
            offset=offset,
        )

    def set_up_scene(self, scene: Scene) -> None:
        self._scene = scene
        scene.add_default_ground_plane()
        cylinder_prim_path = find_unique_string_name(
            initial_name="/World/Cylinder",
            is_unique_fn=lambda x: not is_prim_path_valid(x),
        )
        cylinder_name = find_unique_string_name(
            initial_name="cylinder",
            is_unique_fn=lambda x: not self.scene.object_exists(x),
        )
        self._cube = scene.add(
            DynamicCylinder(
                prim_path=cylinder_prim_path,
                name=cylinder_name,
                position=self._cube_initial_position,
                orientation=self._cube_initial_orientation,
                radius=self._cube_size[0] / 2,  # 지름의 절반: 반지름
                height=self._cube_size[2],
                color=np.array([0.1, 0.4, 0.3]),
            )
        )
        self._task_objects[self._cube.name] = self._cube
        self._robot = self.set_robot()
        scene.add(self._robot)
        self._task_objects[self._robot.name] = self._robot
        self._move_task_objects_to_their_frame()
        return


FINGER_LENGTH = 0.05  # 50mm in meters
MARGIN = 0.03

my_world = World(stage_units_in_meters=1.0)
cylinder_size = np.array([0.06, 0.06, 0.12])  # np.array([지름, 지름(무효), 높이])

orientation = [0.7071, 0, 0.7071, 0]  # x축으로 90도 회전
if orientation is not None:
    hs_offset = 0
elif cylinder_size[2] > FINGER_LENGTH:
    hs_offset = cylinder_size[2] / 2 - FINGER_LENGTH + MARGIN


my_world = World(stage_units_in_meters=1.0)
my_task = PickPlaceCylinder(
    cylinder_size=cylinder_size, cylinder_initial_orientation=orientation
)
my_world.add_task(my_task)
my_world.reset()
task_params = my_task.get_params()
my_fr3 = my_world.scene.get_object(task_params["robot_name"]["value"])
my_controller = PickPlaceController(
    name="pick_place_controller",
    gripper=my_fr3.gripper,
    robot_articulation=my_fr3,
    lift_offset=hs_offset,
)
articulation_controller = my_fr3.get_articulation_controller()

i = 0
reset_needed = False
previous_state = None
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            my_controller.reset()
            reset_needed = False
        observations = my_world.get_observations()
        actions = my_controller.forward(
            picking_position=observations[task_params["cube_name"]["value"]][
                "position"
            ],
            placing_position=observations[task_params["cube_name"]["value"]][
                "target_position"
            ],
            current_joint_positions=observations[task_params["robot_name"]["value"]][
                "joint_positions"
            ],
        )
        if my_controller.is_done():
            current_state = "Done picking and placing"
        else:
            current_state = f"Phase: {my_controller.get_current_event()}"
        if current_state != previous_state:
            print(current_state)
            previous_state = current_state
        articulation_controller.apply_action(actions)
simulation_app.close()
