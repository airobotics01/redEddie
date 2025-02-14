from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import os
import typing
from typing import List, Optional
import carb
import numpy as np

from isaacsim.core.api.world import World
from isaacsim.core.api.robots.robot import Robot
from isaacsim.core.api import tasks
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.prims import get_prim_at_path
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.extensions import get_extension_path_from_name
from isaacsim.core.utils.types import ArticulationAction

from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators import SingleManipulator
import isaacsim.robot.manipulators.controllers as manipulators_controllers
from isaacsim.robot_motion import motion_generation as mg


class FR3RMPFlowController(mg.MotionPolicyController):
    def __init__(
        self,
        name: str,
        robot_articulation: SingleArticulation,
        physics_dt: float = 1.0 / 60.0,
    ) -> None:
        self.rmp_flow_config = (
            mg.interface_config_loader.load_supported_motion_policy_config(
                "FR3", "RMPflow"
            )
        )
        self.rmp_flow = mg.lula.motion_policies.RmpFlow(**self.rmp_flow_config)

        self.articulation_rmp = mg.ArticulationMotionPolicy(
            robot_articulation, self.rmp_flow, physics_dt
        )

        mg.MotionPolicyController.__init__(
            self, name=name, articulation_motion_policy=self.articulation_rmp
        )
        self._default_position, self._default_orientation = (
            self._articulation_motion_policy._robot_articulation.get_world_pose()
        )
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position,
            robot_orientation=self._default_orientation,
        )

    def reset(self):
        super().reset()
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position,
            robot_orientation=self._default_orientation,
        )


class FR3PickPlaceController(manipulators_controllers.PickPlaceController):
    def __init__(
        self,
        name: str,
        gripper: ParallelGripper,
        robot_articulation: SingleArticulation,
        end_effector_initial_height: Optional[float] = None,
        events_dt: Optional[List[float]] = None,
    ) -> None:
        if events_dt is None:
            events_dt = [
                0.005,  # Phase 0
                0.004,  # Phase 1
                0.1,  # Phase 2
                0.05,  # Phase 3
                0.004,  # Phase 4
                0.004,  # Phase 5
                0.004,  # Phase 6
                0.05,  # Phase 7
                0.004,  # Phase 8
                0.008,  # Phase 9
            ]
        super().__init__(
            name=name,
            cspace_controller=FR3RMPFlowController(
                name=name + "_cspace_controller", robot_articulation=robot_articulation
            ),
            gripper=gripper,
            end_effector_initial_height=end_effector_initial_height,
            events_dt=events_dt,
        )


class FR3(Robot):
    def __init__(
        self,
        prim_path: str,
        name: str = "my_fr3",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        end_effector_prim_name: Optional[str] = None,
        gripper_dof_names: Optional[List[str]] = None,
        gripper_open_position: Optional[np.ndarray] = None,
        gripper_closed_position: Optional[np.ndarray] = None,
        deltas: Optional[np.ndarray] = None,
    ) -> None:
        prim = get_prim_at_path(prim_path)
        self._end_effector = None
        self._gripper = None
        self._end_effector_prim_name = end_effector_prim_name
        if not prim.IsValid():
            if usd_path:
                add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
            else:
                assets_root_path = get_assets_root_path()
                if assets_root_path is None:
                    carb.log_error("Could not find Isaac Sim assets folder")
                usd_path = assets_root_path + "/Isaac/Robots/Franka/FR3/fr3.usd"
                add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
            if self._end_effector_prim_name is None:
                self._end_effector_prim_path = prim_path + "/fr3_rightfinger"
            else:
                self._end_effector_prim_path = prim_path + "/" + end_effector_prim_name
            if gripper_dof_names is None:
                gripper_dof_names = ["fr3_finger_joint1", "fr3_finger_joint2"]
            if gripper_open_position is None:
                gripper_open_position = np.array([0.04, 0.04]) / get_stage_units()
            if gripper_closed_position is None:
                gripper_closed_position = np.array([0.0, 0.0])
        else:
            if self._end_effector_prim_name is None:
                self._end_effector_prim_path = prim_path + "/fr3_rightfinger"
            else:
                self._end_effector_prim_path = prim_path + "/" + end_effector_prim_name
            if gripper_dof_names is None:
                gripper_dof_names = ["fr3_finger_joint1", "fr3_finger_joint2"]
            if gripper_open_position is None:
                gripper_open_position = np.array([0.04, 0.04]) / get_stage_units()
            if gripper_closed_position is None:
                gripper_closed_position = np.array([0.0, 0.0])
        super().__init__(
            prim_path=prim_path,
            name=name,
            position=position,
            orientation=orientation,
            articulation_controller=None,
        )
        if gripper_dof_names is not None:
            if deltas is None:
                deltas = np.array([0.04, 0.04]) / get_stage_units()
            self._gripper = ParallelGripper(
                end_effector_prim_path=self._end_effector_prim_path,
                joint_prim_names=gripper_dof_names,
                joint_opened_positions=gripper_open_position,
                joint_closed_positions=gripper_closed_position,
                action_deltas=deltas,
            )
        return

    @property
    def end_effector(self) -> SingleRigidPrim:
        """[summary]

        Returns:
            SingleRigidPrim: [description]
        """
        return self._end_effector

    @property
    def gripper(self) -> ParallelGripper:
        """[summary]

        Returns:
            ParallelGripper: [description]
        """
        return self._gripper

    def initialize(self, physics_sim_view=None) -> None:
        """[summary]"""
        super().initialize(physics_sim_view)
        self._end_effector = SingleRigidPrim(
            prim_path=self._end_effector_prim_path, name=self.name + "_end_effector"
        )
        self._end_effector.initialize(physics_sim_view)
        self._gripper.initialize(
            physics_sim_view=physics_sim_view,
            articulation_apply_action_func=self.apply_action,
            get_joint_positions_func=self.get_joint_positions,
            set_joint_positions_func=self.set_joint_positions,
            dof_names=self.dof_names,
        )
        return

    def post_reset(self) -> None:
        """[summary]"""
        super().post_reset()
        self._gripper.post_reset()
        self._articulation_controller.switch_dof_control_mode(
            dof_index=self.gripper.joint_dof_indicies[0], mode="position"
        )
        self._articulation_controller.switch_dof_control_mode(
            dof_index=self.gripper.joint_dof_indicies[1], mode="position"
        )
        return


class FR3PickPlaceTask(tasks.PickPlace):
    def __init__(
        self,
        name: str = "FR3_pick_place",
        cube_initial_position: Optional[np.ndarray] = None,
        cube_initial_orientation: Optional[np.ndarray] = None,
        target_position: Optional[np.ndarray] = None,
        cube_size: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        super().__init__(
            name=name,
            cube_initial_position=cube_initial_position,
            cube_initial_orientation=cube_initial_orientation,
            target_position=target_position,
            cube_size=cube_size,
            offset=offset,
        )

    def set_robot(self) -> SingleManipulator:
        fr3_prim_path = "/World/FR3"
        fr3_robot_name = "my_fr3"
        fr3_robot = FR3(prim_path=fr3_prim_path, name=fr3_robot_name)
        joints_default_positions = np.array(
            [0.0, -0.3, 0.0, -1.8, 0.0, 1.5, 0.7, 0.04, -0.04]
        )
        fr3_robot.set_joints_default_state(positions=joints_default_positions)
        return fr3_robot


class FR3StackingController(manipulators_controllers.StackingController):
    def __init__(
        self,
        name: str,
        gripper: ParallelGripper,
        robot_articulation: SingleArticulation,
        picking_order_cube_names: List[str],
        robot_observation_name: str,
    ) -> None:
        super().__init__(
            name=name,
            pick_place_controller=FR3PickPlaceController(
                name=name + "_pick_place_controller",
                gripper=gripper,
                robot_articulation=robot_articulation,
            ),
            picking_order_cube_names=picking_order_cube_names,
            robot_observation_name=robot_observation_name,
        )

    def get_current_event(self) -> int:
        _current_phase = self._pick_place_controller.get_current_event()
        _current_cube_name = self._picking_order_cube_names[self._current_cube]

        return _current_cube_name, _current_phase


class FR3StackTask(tasks.Stacking):
    def __init__(
        self,
        name: str = "FR3_stacking",
        cube_initial_positions: np.ndarray = None,
        cube_initial_orientations: Optional[np.ndarray] = None,
        stack_target_position: Optional[np.ndarray] = None,
        cube_size: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        if stack_target_position is None:
            stack_target_position = np.array([0.5, 0.5, 0]) / get_stage_units()
        super().__init__(
            name=name,
            cube_initial_positions=cube_initial_positions,
            cube_initial_orientations=cube_initial_orientations,
            stack_target_position=stack_target_position,
            cube_size=cube_size,
            offset=offset,
        )

    def set_robot(self) -> SingleManipulator:
        fr3_prim_path = "/World/FR3"
        fr3_robot_name = "my_fr3"
        fr3_robot = FR3(prim_path=fr3_prim_path, name=fr3_robot_name)
        joints_default_positions = np.array(
            [0.0, -0.3, 0.0, -1.8, 0.0, 1.5, 0.7, 0.04, -0.04]
        )
        fr3_robot.set_joints_default_state(positions=joints_default_positions)
        return fr3_robot


my_world = World(stage_units_in_meters=1.0)

# 큐브 초기 위치 설정
cube_positions = np.array(
    [
        [0.5, -0.3, 0.0515 / 2.0],
        [0.5, 0.0, 0.0515 / 2.0],
        [0.5, 0.3, 0.0515 / 2.0],
    ]
)
cube_positions[:, 2] = 0.0515 / 2.0  # 큐브 위치 설정 (z축은 객체 크기의 절반)
stack_target_position = np.array([-0.3, 0.6, 0])
cube_size = np.array([0.0515, 0.0515, 0.0515])

my_task = FR3StackTask(
    cube_initial_positions=cube_positions,
    stack_target_position=stack_target_position,
    cube_size=cube_size,
)
my_world.add_task(my_task)
my_world.reset()

robot_name = my_task.get_params()["robot_name"]["value"]
my_fr3 = my_world.scene.get_object(robot_name)

my_controller = FR3StackingController(
    name="FR3_stacking_controller",
    gripper=my_fr3.gripper,
    robot_articulation=my_fr3,
    picking_order_cube_names=["cube_1", "cube", "cube_2"],  # 큐브 순서 지정
    robot_observation_name=my_world.get_task("FR3_stacking").get_params()["robot_name"][
        "value"
    ],
)
articulation_controller = my_fr3.get_articulation_controller()


reset_needed = False
previous_state = None
while simulation_app.is_running():
    my_world.step(render=True)

    if my_world.is_stopped() and not reset_needed:
        reset_needed = True

    if my_world.is_playing():
        if reset_needed or my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()
            reset_needed = False

        observations = my_world.get_observations()
        actions = my_controller.forward(
            observations=observations,
            end_effector_orientation=None,
            end_effector_offset=np.array([0, 0, 0]),
        )
        articulation_controller.apply_action(actions)

        if my_controller.is_done():
            current_state = "Done stacking"
        else:
            current_state = f"Stacking {my_controller.get_current_event()}"

        if current_state != previous_state:
            print(current_state)
            previous_state = current_state

simulation_app.close()
