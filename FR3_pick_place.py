from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
from omni.isaac.core import World
from omni.isaac.franka.controllers.pick_place_controller import PickPlaceController

from omni.isaac.core import World
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.types import ArticulationAction
import omni.isaac.core.tasks as tasks
from omni.isaac.core.articulations import Articulation
from omni.isaac.franka.tasks import PickPlace
from omni.isaac.franka.controllers import RMPFlowController


my_world = World(stage_units_in_meters=1.0)

robot_prim_path = "/World/FR3"
path_to_robot_usd = get_assets_root_path() + "/Isaac/Robots/Franka/FR3/fr3.usd"
add_reference_to_stage(path_to_robot_usd, robot_prim_path)

gripper = (
    ParallelGripper(  # For the finger only the first value is used, second is ignored
        end_effector_prim_path="/World/FR3/fr3_hand",
        joint_prim_names=["fr3_finger_joint1", "fr3_finger_joint2"],
        joint_opened_positions=np.array([0.04, 0.04]),
        joint_closed_positions=np.array([0, 0]),
        action_deltas=np.array([0.04, 0.04]),
    )
)

fr3_robot = Articulation(robot_prim_path)
my_world.scene.add(fr3_robot)


class FR3PickPlaceController(PickPlaceController):
    def __init__(
        self,
        name: str,
        gripper: ParallelGripper,
        robot_articulation: Articulation,
        end_effector_initial_height: float = None,
        events_dt: list[float] = None,
    ):
        super().__init__(
            name,
            gripper,
            robot_articulation,
            end_effector_initial_height,
            events_dt,
        )

    def forward(self, picking_position, placing_position, current_joint_positions):
        # Implement FR3-specific logic here if needed
        return super().forward(
            picking_position, placing_position, current_joint_positions
        )


fr3_pick_place_controller = FR3PickPlaceController(
    "fr3_pick_place",
    gripper,
    fr3_robot,
    0.3,
)


print("controller initiated")


my_task = PickPlace()
my_world.add_task(my_task)


my_world.reset()
task_params = my_task.get_params()


i = 0
reset_needed = False
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            fr3_pick_place_controller.reset()
            reset_needed = False
        observations = my_world.get_observations()
        actions = fr3_pick_place_controller.forward(
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
        if fr3_pick_place_controller.is_done():
            print("done picking and placing")
        fr3_pick_place_controller.apply_action(actions)
simulation_app.close()
