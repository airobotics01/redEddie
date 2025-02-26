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

# Debug drawing extension
import random
from omni.isaac.core.utils.extensions import enable_extension

enable_extension("isaacsim.util.debug_draw")

from isaacsim.util.debug_draw import _debug_draw

import omni.ui as ui
import omni.kit.viewport.utility
import carb


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

        # Drawing variables
        self.custom_timer = 0
        self.point_list = []
        self.draw = None
        # Default drawing settings - will be updated from UI
        self.draw_scale = 1.0
        self.draw_color = (1.0, 0.5, 0.2, 1.0)  # Default color (RGBA)
        self.line_thickness = 5

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

    def setup_post_load(self):
        # Initialize Debug Draw
        self.draw = _debug_draw.acquire_debug_draw_interface()
        return

    def reset_drawing(self):
        """Reset the drawing data"""
        self.custom_timer = 0
        self.point_list = []
        if self.draw:
            self.draw.clear_lines()

    def update_drawing_settings(self, scale=None, color=None, thickness=None):
        """Update the drawing settings from UI"""
        if scale is not None:
            self.draw_scale = scale
        if color is not None:
            self.draw_color = color
        if thickness is not None:
            self.line_thickness = thickness

    def draw_heart_shape(self, observations, target_name):
        """Calculate and draw the heart shape based on current settings"""
        scale_factor = 0.01 * self.draw_scale
        t = self.custom_timer * scale_factor
        x = (16 * np.power(np.sin(t), 3)) * scale_factor
        y = (
            13 * np.cos(t) - 5 * np.cos(2 * t) - 2 * np.cos(3 * t) - np.cos(4 * t)
        ) * scale_factor

        new_pos = observations[target_name]["position"] + [0, x, y]

        # Record points for drawing
        if self.custom_timer % 10 == 0:
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

        self.custom_timer += 1
        return new_pos


class HeartShapeControlWindow(ui.Window):
    def __init__(self, title: str, task: SimpleRoom, **kwargs):
        super().__init__(title, width=375, height=500, **kwargs)
        self.frame.set_build_fn(self._build_fn)
        self.task = task

        # UI control values
        self.size_value = 1.0

        # Store color as RGBA tuple instead of using UI.Color
        self.color_r = 1.0
        self.color_g = 0.5
        self.color_b = 0.2
        self.color_a = 1.0

        self.line_thickness = 2
        self.drawing_active = True

    def _build_fn(self):
        with ui.VStack(spacing=10, height=0):
            ui.Label("Heart Shape Controls", alignment=ui.Alignment.CENTER)

            ui.Spacer(height=20)

            # Size adjustment slider
            with ui.HStack(height=0):
                ui.Label("Size:", width=80)
                slider = ui.FloatSlider(min=0.1, max=5.0, step=0.1)
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
                    ui.Label("R:", width=20)
                    red_slider = ui.FloatSlider(min=0.0, max=1.0, step=0.01)
                    red_slider.model.set_value(self.color_r)
                    red_slider.model.add_value_changed_fn(self._on_color_r_changed)

                # Green
                with ui.HStack(height=0):
                    ui.Label("G:", width=20)
                    green_slider = ui.FloatSlider(min=0.0, max=1.0, step=0.01)
                    green_slider.model.set_value(self.color_g)
                    green_slider.model.add_value_changed_fn(self._on_color_g_changed)

                # Blue
                with ui.HStack(height=0):
                    ui.Label("B:", width=20)
                    blue_slider = ui.FloatSlider(min=0.0, max=1.0, step=0.01)
                    blue_slider.model.set_value(self.color_b)
                    blue_slider.model.add_value_changed_fn(self._on_color_b_changed)

            ui.Spacer(height=20)

            # Color preview
            with ui.HStack(height=0):
                ui.Label("Color Preview:", width=80)
                # Create a small rectangle with the selected color
                with ui.ZStack(width=30, height=20):
                    # We can't use the actual color, so use a rectangular frame as a preview
                    ui.Rectangle(
                        style={
                            "background_color": f"rgba({int(self.color_r*255)}, {int(self.color_g*255)}, {int(self.color_b*255)}, {self.color_a})"
                        }
                    )

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

    def _on_size_changed(self, model):
        self.size_value = model.get_value_as_float()
        print(f"Size adjusted to: {self.size_value:.1f}")

    def _on_thickness_changed(self, model):
        self.line_thickness = model.get_value_as_int()
        print(f"Line thickness adjusted to: {self.line_thickness}")

    def _on_color_r_changed(self, model):
        self.color_r = model.get_value_as_float()
        self._update_color_preview()

    def _on_color_g_changed(self, model):
        self.color_g = model.get_value_as_float()
        self._update_color_preview()

    def _on_color_b_changed(self, model):
        self.color_b = model.get_value_as_float()
        self._update_color_preview()

    def _update_color_preview(self):
        print(
            f"Color changed to: RGBA({self.color_r:.2f}, {self.color_g:.2f}, {self.color_b:.2f}, {self.color_a:.2f})"
        )
        # Force rebuild to update color preview
        self.frame.rebuild()

    def _on_drawing_toggle(self, model):
        self.drawing_active = model.get_value_as_bool()
        print(f"Drawing active: {self.drawing_active}")

    def _apply_settings(self):
        # Create color tuple
        color_tuple = (self.color_r, self.color_g, self.color_b, self.color_a)

        # Update task drawing settings
        self.task.update_drawing_settings(
            scale=self.size_value, color=color_tuple, thickness=self.line_thickness
        )

        print(
            f"Applied settings - Size: {self.size_value:.1f}, "
            f"Thickness: {self.line_thickness}, "
            f"Color: RGBA({self.color_r:.2f}, {self.color_g:.2f}, {self.color_b:.2f}, {self.color_a:.2f})"
        )

    def _reset_drawing(self):
        print("Resetting drawing")
        self.task.reset_drawing()

    def _reset_settings(self):
        self.size_value = 1.0
        self.line_thickness = 5
        self.color_r = 1.0
        self.color_g = 0.5
        self.color_b = 0.2
        self.color_a = 1.0
        self.drawing_active = True

        # Apply the reset settings to the task
        color_tuple = (self.color_r, self.color_g, self.color_b, self.color_a)
        self.task.update_drawing_settings(
            scale=self.size_value, color=color_tuple, thickness=self.line_thickness
        )

        print("Settings reset to default values")
        # Force UI rebuild to show the reset values
        self.frame.rebuild()

    def is_drawing_active(self):
        return self.drawing_active


# Main script execution
my_world = World(stage_units_in_meters=1.0)

# Set camera view
eye_position = [2.0, 0.0, 0.5]
target_position = [0.0, 0.0, 0.5]
camera_prim_path = "/OmniverseKit_Persp"
set_camera_view(
    eye=eye_position, target=target_position, camera_prim_path=camera_prim_path
)

# Create task
my_task = SimpleRoom(name="follow_target_task", target_position=[0.2, 0, 0.7])
my_world.add_task(my_task)
my_world.reset()

# Initialize Debug Draw
my_task.setup_post_load()

# Get task parameters
task_params = my_world.get_task("follow_target_task").get_params()
franka_name = task_params["robot_name"]["value"]
target_name = task_params["target_name"]["value"]
my_franka = my_world.scene.get_object(franka_name)

# Set up controller
my_controller = RMPFlowController(
    name="target_follower_controller", robot_articulation=my_franka
)
articulation_controller = my_franka.get_articulation_controller()

# Create control window and pass the task reference
control_window = HeartShapeControlWindow("Heart Shape Controls", my_task)

# Main simulation loop
reset_needed = False
while simulation_app.is_running():
    my_world.step(render=True)

    if my_world.is_stopped() and not reset_needed:
        reset_needed = True

    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            my_controller.reset()
            my_task.reset_drawing()
            reset_needed = False

        observations = my_world.get_observations()

        # Only update drawing if active in UI
        if control_window.is_drawing_active():
            # Draw heart shape and get new position
            new_pos = my_task.draw_heart_shape(observations, target_name)

            # Update robot controller
            actions = my_controller.forward(
                target_end_effector_position=new_pos,
                target_end_effector_orientation=observations[target_name][
                    "orientation"
                ],
            )

            # Apply actions to the robot
            articulation_controller.apply_action(actions)

simulation_app.close()
