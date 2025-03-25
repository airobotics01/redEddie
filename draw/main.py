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
from franka import FR3
from modules.visualization.trajectory_drawer import TrajectoryDrawer

import isaacsim.core.api.tasks as tasks
import numpy as np
import omni.ui as ui
import omni.kit.viewport.utility
import carb
import random
from typing import List, Optional

from korean import find_paths_by_name, generate_korean_character


# Debug drawing extension
enable_extension("isaacsim.util.debug_draw")
from isaacsim.util.debug_draw import _debug_draw

from modules.robot_control.fr3_follow import FR3Follow


# End Effector 위치 추적 함수
def get_end_effector_position():
    """FR3 로봇의 엔드 이펙터 위치 가져오기"""
    stage = get_current_stage()
    ee_prim = get_prim_at_path("/World/FR3/fr3_hand_tcp")
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
    eye_position = [1.0, 0.0, 0.8]
    target_position = [0.0, 0.0, 0.0]
    camera_prim_path = "/OmniverseKit_Persp"
    set_camera_view(
        eye=eye_position, target=target_position, camera_prim_path=camera_prim_path
    )

    # 초기화
    character_name = "ㄹ"  # 그릴 한글 문자
    original_position = [0.5, 0, 0.2]  # 시작 위치
    my_task = FR3Follow(name="drawing_task", target_position=original_position)
    my_world.add_task(my_task)
    my_world.reset()

    ee_drawer = TrajectoryDrawer()
    ee_drawer.initialize()

    # Task 파라미터 가져오기
    task_params = my_world.get_task("drawing_task").get_params()
    franka_name = task_params["robot_name"]["value"]
    target_name = task_params["target_name"]["value"]
    my_franka = my_world.scene.get_object(franka_name)

    # 기본 관절 위치 설정
    joints_default_positions = np.array(
        [0.0, -0.3, 0.0, -1.8, 0.0, 1.5, 0.7, 0.04, 0.04]
    )
    my_franka.set_joints_default_state(positions=joints_default_positions)

    # 컨트롤러 설정
    my_controller = RMPFlowController(
        name="target_follower_controller", robot_articulation=my_franka
    )
    articulation_controller = my_franka.get_articulation_controller()

    # 메인 시뮬레이션 루프
    reset_needed = False
    tick = 0
    current_stroke = 0
    while simulation_app.is_running():
        my_world.step(render=True)

        if my_world.is_stopped() and not reset_needed:
            reset_needed = True

        if my_world.is_playing():
            if reset_needed:
                my_world.reset()
                my_controller.reset()
                reset_needed = False
                current_stroke = 0
                tick = 0  # 새로운 획을 위해 tick 초기화

            observations = my_world.get_observations()
            tick += 1

            # 현재 엔드 이펙터 위치 가져오기
            ee_pos = get_end_effector_position()
            if ee_pos is not None:
                ee_drawer.update_drawing(ee_pos)

                # 한글 문자 궤적 생성
                trajectory, is_stroke_complete = generate_korean_character(
                    tick,
                    character_name,
                    current_stroke=current_stroke,
                    ee_pos=ee_pos,
                    original_position=np.array(original_position),
                )
                if trajectory is not None:
                    # 타겟 위치 설정
                    my_task.set_cube_pose(trajectory)

                    # 로봇 컨트롤러 업데이트
                    actions = my_controller.forward(
                        target_end_effector_position=trajectory,
                        target_end_effector_orientation=observations[target_name][
                            "orientation"
                        ],
                    )

                    # 로봇에 액션 적용
                    articulation_controller.apply_action(actions)

                # 현재 획이 완료되었는지 확인
                if is_stroke_complete:
                    current_stroke += 1
                    print(f"[STROKE UPDATE] current_stroke: {current_stroke}")
                    tick = 0  # 새로운 획을 위해 tick 초기화

            # 디버깅 그리기
            ee_pos = get_end_effector_position()
            if ee_pos is not None:
                ee_drawer.update_drawing(ee_pos)

    simulation_app.close()


if __name__ == "__main__":
    main()
