from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import World
from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.string import find_unique_string_name
from isaacsim.core.utils.prims import is_prim_path_valid, get_prim_at_path
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.core.utils.extensions import enable_extension
from omni.kit.viewport.utility.camera_state import ViewportCameraState
from pxr import UsdGeom

from controllers.rmpflow_controller import RMPFlowController
from tasks.follow_target import FollowTarget
from franka import FR3

import isaacsim.core.api.tasks as tasks
import numpy as np
import omni.ui as ui
import omni.kit.viewport.utility
import carb
import random
from typing import List, Optional

# Debug drawing extension
enable_extension("isaacsim.util.debug_draw")
from isaacsim.util.debug_draw import _debug_draw


# Task 클래스
class FR3DrawHeart(tasks.FollowTarget):
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
        self.draw_scale = 0.0
        self.tick = 0
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

    def get_robot(self):
        """로봇 객체 반환"""
        return self._robot

    def get_cube_pose(self):
        """큐브 위치 반환"""
        cube_position, cube_orientation = self._scene.get_object(
            self._target_name
        ).get_world_pose()

        return cube_position

    def generate_heart(self):
        """하트 모양 생성"""
        self.tick += 1
        self.draw_scale = 1.0
        scale_factor = 0.01 * self.draw_scale
        t = self.tick * scale_factor
        x = (16 * np.power(np.sin(t), 3)) * scale_factor
        y = (
            13 * np.cos(t) - 5 * np.cos(2 * t) - 2 * np.cos(3 * t) - np.cos(4 * t)
        ) * scale_factor

        original_position = self.get_cube_pose()
        new_pos = original_position + [0, x, y]
        return new_pos


class TrajectoryDrawer:
    """궤적을 시각화하기 위한 Debug 클래스"""

    def __init__(self):
        self.draw = None
        self.point_list = []
        self.colors = (0, 0, 0, 1)

    def initialize(self):
        """Debug Draw 인터페이스 초기화"""
        self.draw = _debug_draw.acquire_debug_draw_interface()

    def update_drawing(self, position, draw_offset=None):
        """
        하트 궤적을 그리기 위한 포인트 업데이트 및 시각화

        Args:
            position: Task에서 계산된 현재 위치
            draw_offset: 그리기 위치 오프셋 (선택 사항)
        """
        if draw_offset is None:
            draw_offset = [0.05, 0, 0]

        # 포인트 리스트에 현재 위치 추가 (주기적으로)
        self.point_list.append(tuple(position + draw_offset))
        self.colors = (
            random.uniform(0, 1),
            random.uniform(0, 1),
            random.uniform(0, 1),
            1,
        )  # RGBa

        # 가끔씩 라인 정리
        if len(self.point_list) % 10 == 0:
            self.draw.clear_lines()

        # 경로 그리기
        if len(self.point_list) != 0:
            self.draw.draw_lines_spline(self.point_list, self.colors, 5, False)

        # 버퍼 크기 관리
        if len(self.point_list) > 70:
            del self.point_list[0]

    def reset_drawing(self):
        """하트 궤적 초기화"""
        self.point_list = []


# End Effector 위치 추적 함수
def get_end_effector_position():
    """FR3 로봇의 엔드 이펙터 위치 가져오기"""
    stage = get_current_stage()
    ee_prim = get_prim_at_path("/World/FR3/fr3_rightfinger")
    if ee_prim:
        xform = UsdGeom.Xformable(ee_prim)
        world_transform = xform.ComputeLocalToWorldTransform(0)
        return world_transform.ExtractTranslation()
    return None


# 메인 스크립트 실행
def main():
    # 월드 생성
    my_world = World(stage_units_in_meters=1.0)

    # 카메라 뷰 설정
    eye_position = [2.0, 0.0, 0.5]
    target_position = [0.0, 0.0, 0.5]
    camera_prim_path = "/OmniverseKit_Persp"
    set_camera_view(
        eye=eye_position, target=target_position, camera_prim_path=camera_prim_path
    )

    # 초기화
    my_task = FR3DrawHeart(name="heart_drawing_task", target_position=[0.5, 0, 0.7])
    my_world.add_task(my_task)
    my_world.reset()

    heart_drawer = TrajectoryDrawer()
    heart_drawer.initialize()
    ee_drawer = TrajectoryDrawer()
    ee_drawer.initialize()

    # Task 파라미터 가져오기
    task_params = my_world.get_task("heart_drawing_task").get_params()
    franka_name = task_params["robot_name"]["value"]
    target_name = task_params["target_name"]["value"]
    my_franka = my_world.scene.get_object(franka_name)

    # 컨트롤러 설정
    my_controller = RMPFlowController(
        name="target_follower_controller", robot_articulation=my_franka
    )
    articulation_controller = my_franka.get_articulation_controller()

    # 메인 시뮬레이션 루프
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

            new_pos = my_task.generate_heart()

            # 로봇 컨트롤러 업데이트
            actions = my_controller.forward(
                target_end_effector_position=new_pos,
                target_end_effector_orientation=observations[target_name][
                    "orientation"
                ],
            )

            # 로봇에 액션 적용
            articulation_controller.apply_action(actions)

            # 디버깅 그리기
            heart_drawer.update_drawing(new_pos)
            ee_pos = get_end_effector_position()
            if ee_pos is not None:
                ee_drawer.update_drawing(position=ee_pos, draw_offset=[0, 0, 0])

    simulation_app.close()


if __name__ == "__main__":
    main()
