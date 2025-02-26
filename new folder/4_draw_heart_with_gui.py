from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import World
from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.string import find_unique_string_name
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.core.utils.stage import add_reference_to_stage
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.core.utils.extensions import enable_extension
from omni.kit.viewport.utility.camera_state import ViewportCameraState

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
class FrankaRobotTask(tasks.FollowTarget):
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


# Drawing 클래스
class HeartShapeDrawing:
    """하트 모양 생성 및 그리기를 담당하는 클래스"""

    def __init__(self):
        # Drawing variables
        self.custom_timer = 0
        self._frame_counter = 0
        self.timer_speed = 1.0
        self.point_list = []
        self.draw = None
        # Default drawing settings
        self.draw_scale = 1.0
        self.draw_color = (1.0, 0.5, 0.2, 1.0)  # Default color (RGBA)
        self.line_thickness = 5
        self.is_active = True

    def setup_post_load(self):
        """디버그 드로잉 인터페이스 초기화"""
        self.draw = _debug_draw.acquire_debug_draw_interface()
        return

    def reset_drawing(self):
        """그리기 데이터 초기화"""
        self.custom_timer = 0
        self._frame_counter = 0
        self.point_list = []
        if self.draw:
            self.draw.clear_lines()

    def update_drawing_settings(
        self, timer_speed=None, scale=None, color=None, thickness=None
    ):
        """UI에서 그리기 설정 업데이트"""
        if timer_speed is not None:
            self.timer_speed = timer_speed
        if scale is not None:
            self.draw_scale = scale
        if color is not None:
            self.draw_color = color
        if thickness is not None:
            self.line_thickness = thickness

    def draw_heart_shape(self, observations, target_name):
        """현재 설정을 기반으로 하트 모양 계산 및 그리기"""
        if not self.is_active:
            return observations[target_name]["position"]

        scale_factor = 0.01 * self.draw_scale
        t = self.custom_timer * scale_factor
        x = (16 * np.power(np.sin(t), 3)) * scale_factor
        y = (
            13 * np.cos(t) - 5 * np.cos(2 * t) - 2 * np.cos(3 * t) - np.cos(4 * t)
        ) * scale_factor

        new_pos = observations[target_name]["position"] + [0, x, y]

        # Record points for drawing
        self._frame_counter += 1
        if self._frame_counter % 10 == 0:
            self.point_list.append(tuple(new_pos + [0.05, 0, 0]))
            if self.draw:
                self.draw.clear_lines()  # cleanup some lines every 10 tick

        # Draw lines if we have points
        if len(self.point_list) != 0 and self.draw:
            self.draw.draw_lines_spline(
                self.point_list, self.draw_color, self.line_thickness, False
            )

        # Maintain a reasonable buffer size
        if len(self.point_list) > 70:
            del self.point_list[0]

        # Use timer_speed to adjust the increment rate
        self.custom_timer += self.timer_speed
        return new_pos

    def set_active(self, active: bool):
        """그리기 활성화/비활성화 설정"""
        self.is_active = active


# UI 클래스
class HeartShapeControlWindow(ui.Window):
    """하트 모양 그리기 제어를 위한 UI 윈도우 클래스"""

    def __init__(self, title: str, drawing: HeartShapeDrawing, **kwargs):
        super().__init__(title, width=375, height=500, **kwargs)
        self.frame.set_build_fn(self._build_fn)
        self.drawing = drawing

        # UI control values
        self.timer_speed = 1.0
        self.size_value = 1.0
        self.color_r = 1.0
        self.color_g = 0.5
        self.color_b = 0.2
        self.color_a = 1.0
        self.line_thickness = 5
        self.drawing_active = True

    def _build_fn(self):
        with ui.VStack(spacing=10, height=0):
            ui.Label("Heart Shape Controls", alignment=ui.Alignment.CENTER)

            ui.Spacer(height=20)

            # Timer speed adjustment slider (NEW)
            with ui.HStack(height=0):
                ui.Label("Timer Speed:", width=80)
                timer_slider = ui.FloatSlider(min=0.1, max=5.0, step=0.1)
                timer_slider.model.set_value(self.timer_speed)
                timer_slider.model.add_value_changed_fn(self._on_timer_speed_changed)

            ui.Spacer(height=10)

            # Size adjustment slider
            with ui.HStack(height=0):
                ui.Label("Size:", width=80)
                slider = ui.FloatSlider(min=0.2, max=3.0, step=0.1)
                slider.model.set_value(self.size_value)
                slider.model.add_value_changed_fn(self._on_size_changed)

            ui.Spacer(height=10)

            # Line thickness slider
            with ui.HStack(height=0):
                ui.Label("Thickness:", width=80)
                thickness_slider = ui.IntSlider(min=1, max=20)
                thickness_slider.model.set_value(self.line_thickness)
                thickness_slider.model.add_value_changed_fn(self._on_thickness_changed)

            ui.Spacer(height=10)

            # Color sliders (RGB)
            with ui.VStack(spacing=5, height=0):
                ui.Label("Color:", width=80)

                # Red
                with ui.HStack(height=0):
                    ui.Label("R:", width=80)
                    red_slider = ui.FloatSlider(min=0.0, max=1.0, step=0.01)
                    red_slider.model.set_value(self.color_r)
                    red_slider.model.add_value_changed_fn(self._on_color_r_changed)

                # Green
                with ui.HStack(height=0):
                    ui.Label("G:", width=80)
                    green_slider = ui.FloatSlider(min=0.0, max=1.0, step=0.01)
                    green_slider.model.set_value(self.color_g)
                    green_slider.model.add_value_changed_fn(self._on_color_g_changed)

                # Blue
                with ui.HStack(height=0):
                    ui.Label("B:", width=80)
                    blue_slider = ui.FloatSlider(min=0.0, max=1.0, step=0.01)
                    blue_slider.model.set_value(self.color_b)
                    blue_slider.model.add_value_changed_fn(self._on_color_b_changed)

            ui.Spacer(height=20)

            # Drawing toggle
            with ui.HStack(height=0):
                ui.Label("Drawing:", width=80)
                toggle = ui.CheckBox()
                toggle.model.set_value(self.drawing_active)
                toggle.model.add_value_changed_fn(self._on_drawing_toggle)

            ui.Spacer(height=20)

            # Control buttons
            with ui.HStack(height=0, spacing=10):
                ui.Button("Apply", clicked_fn=self._apply_settings)
                ui.Button("Reset Drawing", clicked_fn=self._reset_drawing)
                ui.Button("Reset Settings", clicked_fn=self._reset_settings)
                ui.Button("Close", clicked_fn=lambda: self.destroy())

    def _on_timer_speed_changed(self, model):
        """타이머 속도 변경 핸들러"""
        self.timer_speed = model.get_value_as_float()
        print(f"Timer speed adjusted to: {self.timer_speed:.1f}")

    def _on_size_changed(self, model):
        self.size_value = model.get_value_as_float()
        print(f"Size adjusted to: {self.size_value:.1f}")

    def _on_thickness_changed(self, model):
        self.line_thickness = model.get_value_as_int()
        print(f"Line thickness adjusted to: {self.line_thickness}")

    def _on_color_r_changed(self, model):
        self.color_r = model.get_value_as_float()

    def _on_color_g_changed(self, model):
        self.color_g = model.get_value_as_float()

    def _on_color_b_changed(self, model):
        self.color_b = model.get_value_as_float()

    def _on_drawing_toggle(self, model):
        self.drawing_active = model.get_value_as_bool()
        self.drawing.set_active(self.drawing_active)
        print(f"Drawing active: {self.drawing_active}")

    def _apply_settings(self):
        # Create color tuple
        color_tuple = (self.color_r, self.color_g, self.color_b, self.color_a)

        # Update drawing settings
        self.drawing.update_drawing_settings(
            scale=self.size_value,
            color=color_tuple,
            thickness=self.line_thickness,
            timer_speed=self.timer_speed,
        )

        print(
            f"Applied settings - Timer Speed: {self.timer_speed:.1f}, "
            f"Size: {self.size_value:.1f}, "
            f"Thickness: {self.line_thickness}, "
            f"Color: RGBA({self.color_r:.2f}, {self.color_g:.2f}, {self.color_b:.2f}, {self.color_a:.2f})"
        )

    def _reset_drawing(self):
        print("Resetting drawing")
        self.drawing.reset_drawing()

    def _reset_settings(self):
        self.timer_speed = 1.0
        self.size_value = 1.0
        self.line_thickness = 5
        self.color_r = 1.0
        self.color_g = 0.5
        self.color_b = 0.2
        self.color_a = 1.0
        self.drawing_active = True
        self.drawing.set_active(True)

        # Apply the reset settings to the drawing
        color_tuple = (self.color_r, self.color_g, self.color_b, self.color_a)
        self.drawing.update_drawing_settings(
            timer_speed=self.timer_speed,
            scale=self.size_value,
            color=color_tuple,
            thickness=self.line_thickness,
        )

        print("Settings reset to default values")
        # Force UI rebuild to show the reset values
        self.frame.rebuild()


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

    # Task 생성
    my_task = FrankaRobotTask(name="follow_target_task", target_position=[0.2, 0, 0.5])
    my_world.add_task(my_task)
    my_world.reset()

    # 드로잉 객체 생성
    heart_drawing = HeartShapeDrawing()
    heart_drawing.setup_post_load()

    # Task 파라미터 가져오기
    task_params = my_world.get_task("follow_target_task").get_params()
    franka_name = task_params["robot_name"]["value"]
    target_name = task_params["target_name"]["value"]
    my_franka = my_world.scene.get_object(franka_name)

    # 컨트롤러 설정
    my_controller = RMPFlowController(
        name="target_follower_controller", robot_articulation=my_franka
    )
    articulation_controller = my_franka.get_articulation_controller()

    # 제어 윈도우 생성
    control_window = HeartShapeControlWindow("Heart Shape Controls", heart_drawing)

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
                heart_drawing.reset_drawing()
                reset_needed = False

            observations = my_world.get_observations()

            # 하트 모양 그리기 및 새 위치 가져오기
            new_pos = heart_drawing.draw_heart_shape(observations, target_name)

            # 로봇 컨트롤러 업데이트
            actions = my_controller.forward(
                target_end_effector_position=new_pos,
                target_end_effector_orientation=observations[target_name][
                    "orientation"
                ],
            )

            # 로봇에 액션 적용
            articulation_controller.apply_action(actions)

    simulation_app.close()


if __name__ == "__main__":
    main()
