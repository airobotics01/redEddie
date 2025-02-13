from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import os
import numpy as np

from isaacsim.core.api.world import World
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage
import isaacsim.robot.manipulators.controllers as manipulators_controllers
from isaacsim.core.prims import SingleArticulation
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot_motion import motion_generation as mg
from isaacsim.core.utils.extensions import get_extension_path_from_name
from isaacsim.core.api.tasks import PickPlace
from isaacsim.robot.manipulators import SingleManipulator


class FR3RMPFlowController(mg.MotionPolicyController):
    def __init__(
        self,
        name: str,
        robot_articulation: SingleArticulation,
        end_effector_frame_name: str = "fr3_hand",
        physics_dt: float = 1.0 / 60.0,
    ) -> None:
        mg_extension_path = get_extension_path_from_name(
            "isaacsim.robot_motion.motion_generation"
        )
        rmp_config_dir = os.path.join(
            mg_extension_path, "motion_policy_configs", "FR3", "rmpflow"
        )
        self.rmpflow = mg.lula.motion_policies.RmpFlow(
            robot_description_path=os.path.join(
                rmp_config_dir, "fr3_robot_description.yaml"
            ),
            urdf_path=os.path.join(
                mg_extension_path, "motion_policy_configs", "FR3", "fr3.urdf"
            ),
            rmpflow_config_path=os.path.join(rmp_config_dir, "fr3_rmpflow_config.yaml"),
            end_effector_frame_name=end_effector_frame_name,
            maximum_substep_size=0.00334,
        )
        self.articulation_rmp = mg.ArticulationMotionPolicy(
            robot_articulation, self.rmpflow, physics_dt
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
        end_effector_initial_height: float = None,
        events_dt: list[float] = None,
    ) -> None:
        if events_dt is None:
            events_dt = [
                0.005,  # Phase 0
                0.002,  # Phase 1
                0.1,  # Phase 2
                0.05,  # Phase 3
                0.002,  # Phase 4
                0.005,  # Phase 5
                0.002,  # Phase 6
                0.05,  # Phase 7
                0.002,  # Phase 8
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


class FR3PickPlaceTask(PickPlace):
    def __init__(
        self,
        name: str = "FR3_pick_place",
        cube_initial_position: np.ndarray = None,
        cube_initial_orientation: np.ndarray = None,
        target_position: np.ndarray = None,
        offset: np.ndarray = None,
    ) -> None:
        super().__init__(
            name=name,
            cube_initial_position=cube_initial_position,
            cube_initial_orientation=cube_initial_orientation,
            target_position=target_position,
            cube_size=np.array([0.0515, 0.0515, 0.0515]),
            offset=offset,
        )

    def set_robot(self) -> SingleManipulator:
        robot_prim_path = "/World/FR3"
        path_to_robot_usd = get_assets_root_path() + "/Isaac/Robots/Franka/FR3/fr3.usd"
        add_reference_to_stage(usd_path=path_to_robot_usd, prim_path=robot_prim_path)

        gripper = ParallelGripper(
            end_effector_prim_path="/World/FR3/fr3_hand",
            joint_prim_names=["fr3_finger_joint1", "fr3_finger_joint2"],
            joint_opened_positions=np.array([0.04, 0.04]),
            joint_closed_positions=np.array([0, 0]),
            action_deltas=np.array([0.04, 0.04]),
        )

        fr3_robot = SingleManipulator(
            prim_path=robot_prim_path,
            end_effector_prim_name="fr3_hand",
            name="my_fr3",
            gripper=gripper,
        )
        return fr3_robot


my_world = World(stage_units_in_meters=1.0)
target_position = np.array([-0.3, 0.6, 0])
target_position[2] = 0.0515 / 2.0
my_task = FR3PickPlaceTask(target_position=target_position)
my_world.add_task(my_task)
my_world.reset()

fr3_robot = my_task.set_robot()
fr3_robot.initialize()
gripper = fr3_robot.gripper

my_controller = FR3PickPlaceController(
    name="FR3_controller",
    gripper=gripper,
    robot_articulation=fr3_robot,
    end_effector_initial_height=0.3,
)

task_params = my_world.get_task("FR3_pick_place").get_params()
articulation_controller = fr3_robot.get_articulation_controller()
reset_needed = False

while simulation_app.is_running():
    my_world.step(render=True)

    if my_world.is_stopped() and not reset_needed:
        reset_needed = True

    if my_world.is_playing():
        if reset_needed or my_world.current_time_step_index == 0:
            my_task.cleanup()
            my_world.reset()
            fr3_robot = my_task.set_robot()
            fr3_robot.initialize()
            gripper = fr3_robot.gripper

            my_controller = FR3PickPlaceController(
                name="FR3_controller",
                gripper=gripper,
                robot_articulation=fr3_robot,
                end_effector_initial_height=0.3,
            )
            my_task.post_reset()
            articulation_controller = fr3_robot.get_articulation_controller()
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
            end_effector_offset=np.array([0, 0, 0.0925]),
        )

        if my_controller.is_done():
            print("Done picking and placing")
        else:
            print(f"Phase: {my_controller.get_current_event()}")

        articulation_controller.apply_action(actions)

simulation_app.close()
