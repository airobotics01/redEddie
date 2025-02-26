from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import World

from controllers.rmpflow_controller import RMPFlowController
from tasks.follow_target import FollowTarget


import numpy as np
from typing import List, Optional
from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.string import find_unique_string_name
from isaacsim.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.nucleus import get_assets_root_path

import isaacsim.core.api.tasks as tasks
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.string import find_unique_string_name
from franka import FR3

from isaacsim.core.utils.viewports import set_camera_view
from omni.kit.viewport.utility.camera_state import ViewportCameraState

# TODO: Library for drawing shape
import random
from omni.isaac.core.utils.extensions import enable_extension

enable_extension("isaacsim.util.debug_draw")

from isaacsim.util.debug_draw import _debug_draw


class SimpleRoom(tasks.FollowTarget):
    def __init__(
        self,
        name: str = "fr3_heart_shape",
        target_prim_path: Optional[str] = None,
        target_name: Optional[str] = None,
        target_position: Optional[np.ndarray] = None,
        target_orientation: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
        franka_prim_path: Optional[str] = None,
        franka_robot_name: Optional[str] = None,
    ):
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

        # TODO: Initialize needed variables for drawing
        self.custom_timer = 0
        self.point_list = []
        self.color_list = []
        self.colors = (0, 0, 0, 1)

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

    def set_up_scene(self, scene: Scene) -> None:
        self._scene = scene
        assets_root_path = get_assets_root_path()
        add_reference_to_stage(
            usd_path=f"{assets_root_path}/Isaac/Environments/Simple_Room/simple_room.usd",
            prim_path="/World/SimpleRoom",
        )
        if self._target_orientation is None:
            self._target_orientation = euler_angles_to_quat(
                np.array([-np.pi, 0, np.pi])
            )
        if self._target_prim_path is None:
            self._target_prim_path = find_unique_string_name(
                initial_name="/World/TargetCube",
                is_unique_fn=lambda x: not is_prim_path_valid(x),
            )
        if self._target_name is None:
            self._target_name = find_unique_string_name(
                initial_name="target",
                is_unique_fn=lambda x: not self.scene.object_exists(x),
            )
        self.set_params(
            target_prim_path=self._target_prim_path,
            target_position=self._target_position,
            target_orientation=self._target_orientation,
            target_name=self._target_name,
        )
        self._robot = self.set_robot()
        scene.add(self._robot)
        self._task_objects[self._robot.name] = self._robot
        self._move_task_objects_to_their_frame()
        return

    def setup_post_load(self):  # 해당 함수는 task 클래스에 없는 함수라 선언해도 됨.
        # TODO: Initialize Debug Draw
        self.draw = (
            _debug_draw.acquire_debug_draw_interface()
        )  # drawing tool initialized after world is setup

        return


def outer(func):
    def wrapper(*args):
        print("****")
        func(*args)
        print("****")

    return wrapper


@outer
def print_this(*args):
    for arg in args:
        print(arg)


my_world = World(stage_units_in_meters=1.0)


eye_position = [0.0, 2.0, 0.5]  # 카메라의 위치 (x, y, z)
target_position = [0.0, 0.0, 0.5]  # 카메라가 바라보는 대상의 위치 (x, y, z)
camera_prim_path = "/OmniverseKit_Persp"
set_camera_view(
    eye=eye_position, target=target_position, camera_prim_path=camera_prim_path
)


my_task = SimpleRoom(name="follow_target_task")

my_world.add_task(my_task)
my_world.reset()
# TODO: Initialize Debug Draw
my_task.setup_post_load()
task_params = my_world.get_task("follow_target_task").get_params()
franka_name = task_params["robot_name"]["value"]
target_name = task_params["target_name"]["value"]
my_franka = my_world.scene.get_object(franka_name)
my_controller = RMPFlowController(
    name="target_follower_controller", robot_articulation=my_franka
)
articulation_controller = my_franka.get_articulation_controller()
reset_needed = False
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
        # TODO: Heart Shape Path Creation
        my_task.custom_timer += 1  # increment our timer ticks
        scale_factor = 0.01
        t = my_task.custom_timer * scale_factor
        x = (16 * np.power(np.sin(t), 3)) * scale_factor
        y = (
            13 * np.cos(t) - 5 * np.cos(2 * t) - 2 * np.cos(3 * t) - np.cos(4 * t)
        ) * scale_factor
        print_this(x, y, observations)
        new_pos = observations[target_name]["position"] + [
            0,
            x,
            y,
        ]

        actions = my_controller.forward(
            target_end_effector_position=new_pos,  # observastions[self._task_params["target_name"]["value"]]["position"],
            target_end_effector_orientation=observations[
                task_params["target_name"]["value"]
            ]["orientation"],
        )

        # TODO: Start Drawing the shape
        if my_task.custom_timer % 10 == 0:
            my_task.point_list.append(
                tuple(new_pos + [0.05, 0, 0])
            )  # add 0.05 to move the drawing a bit forward
            my_task.colors = (
                random.uniform(0, 1),
                random.uniform(0, 1),
                random.uniform(0, 1),
                1,
            )  # RGBa
            my_task.draw.clear_lines()  # cleanup some lines every 10 tick

        if len(my_task.point_list) != 0:
            my_task.draw.draw_lines_spline(my_task.point_list, my_task.colors, 5, False)
            pass

        if len(my_task.point_list) > 70:
            del my_task.point_list[0]  # a buffer list main

        articulation_controller.apply_action(actions)

simulation_app.close()
