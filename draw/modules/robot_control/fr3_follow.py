import numpy as np
from typing import Optional
import isaacsim.core.api.tasks as tasks
from isaacsim.core.utils.string import find_unique_string_name
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.api.scenes.scene import Scene
from omni.isaac.nucleus import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.prims import SingleXFormPrim

from franka import FR3


class FR3Follow(tasks.FollowTarget):
    """FR3 로봇 제어를 위한 기본 Task 클래스"""

    def __init__(
        self,
        name: str = "fr3_task",
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
        self._scene = None
        self._robot = None
        return

    def set_robot(self) -> FR3:
        """로봇 객체 생성 및 설정"""
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
        """씬 설정 및 로봇, 타겟 추가"""
        self._scene = scene
        assets_root_path = get_assets_root_path()
        add_reference_to_stage(
            usd_path=f"{assets_root_path}/Isaac/Environments/Simple_Room/simple_room.usd",
            prim_path="/World/SimpleRoom",
        )
        if self._target_orientation is None:
            # x축 기준 180도 회전을 원래대로 되돌리는 쿼터니언 [w, x, y, z]
            self._target_orientation = np.array(
                euler_angles_to_quat(np.array([np.pi, 0.0, 0.0]))
            )  # x축 180도 회전을 원래대로
        if self._target_prim_path is None:
            self._target_prim_path = find_unique_string_name(
                initial_name="/World/motion_commander_target",
                is_unique_fn=lambda x: not is_prim_path_valid(x),
            )
        if self._target_name is None:
            self._target_name = find_unique_string_name(
                initial_name="target",
                is_unique_fn=lambda x: not self.scene.object_exists(x),
            )

        # 타겟 큐브 생성 및 설정
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

    def set_params(
        self,
        target_prim_path: Optional[str] = None,
        target_name: Optional[str] = None,
        target_position: Optional[np.ndarray] = None,
        target_orientation: Optional[np.ndarray] = None,
    ) -> None:
        """타겟 파라미터 설정"""
        if target_prim_path is not None:
            if self._target is not None:
                del self._task_objects[self._target.name]
            if is_prim_path_valid(target_prim_path):
                self._target = self.scene.add(
                    SingleXFormPrim(
                        prim_path=target_prim_path,
                        position=target_position,
                        orientation=target_orientation,
                        name=target_name,
                    )
                )
            else:
                self._target = self.scene.add(
                    VisualCuboid(
                        name=target_name,
                        prim_path=target_prim_path,
                        position=target_position,
                        orientation=target_orientation,
                        color=np.array([0.15, 0.15, 0.15]),  # 회색으로 설정
                        size=0.01,  # 크기 설정
                    )
                )
            self._task_objects[self._target.name] = self._target
        else:
            self._target.set_local_pose(
                position=target_position, orientation=target_orientation
            )
        return

    def get_robot(self):
        """로봇 객체 반환"""
        return self._robot

    def get_cube_pose(self):
        """큐브 위치 반환"""
        cube_position, cube_orientation = self._scene.get_object(
            self._target_name
        ).get_world_pose()
        return cube_position

    def set_cube_pose(self, position, orientation=None):
        """큐브 위치 설정"""
        if orientation is None:
            orientation = (
                self._target_orientation
            )  # 회전 정보가 없으면 원래 회전 정보 사용
        self._scene.get_object(self._target_name).set_world_pose(position, orientation)
