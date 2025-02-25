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

from isaacsim.core.utils.viewports import set_camera_view
from omni.kit.viewport.utility.camera_state import ViewportCameraState


class SimpleRoom(FollowTarget):
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
        actions = my_controller.forward(
            target_end_effector_position=observations[target_name]["position"],
            target_end_effector_orientation=observations[target_name]["orientation"],
        )

        articulation_controller.apply_action(actions)

simulation_app.close()
