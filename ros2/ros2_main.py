from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})


from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api import World
from omni.isaac.nucleus import get_assets_root_path


import numpy as np
from franka import FR3
from ros2_actiongraph import subscribe_joint_command_with_action_graph
from tasks.pick_place import PickPlace
from controllers.pick_place_controller import PickPlaceController

FINGER_LENGTH = 0.05  # 50mm in meters
MARGIN = 0.02

my_world = World()

assets_root_path = get_assets_root_path()

# add_reference_to_stage(
#     usd_path=f"{assets_root_path}/Isaac/Environments/Simple_Room/simple_room.usd",
#     prim_path="/World/SimpleRoom",
# )
# add_reference_to_stage(
#     usd_path=f"{assets_root_path}/Isaac/Robots/Franka/FR3/fr3.usd",
#     prim_path="/World/FR3",
# )

cube_size = np.array([0.17, 0.0515, 0.185])
my_task = PickPlace(cube_size=cube_size)
my_world.add_task(my_task)
my_world.reset()

if cube_size[2] > FINGER_LENGTH:
    hs_offset = cube_size[2] / 2 - FINGER_LENGTH + MARGIN

task_params = my_task.get_params()
my_fr3 = my_world.scene.get_object(task_params["robot_name"]["value"])
my_controller = PickPlaceController(
    name="pick_place_controller",
    gripper=my_fr3.gripper,
    robot_articulation=my_fr3,
    lift_offset=hs_offset,
)

articulation_controller = my_fr3.get_articulation_controller()


subscribe_joint_command_with_action_graph()

simulation_app.update()

my_world.reset()
reset_needed = False
previous_state = None
while simulation_app.is_running():
    simulation_app.update()

    if my_world.is_stopped() and not reset_needed:
        reset_needed = True

    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            my_controller.reset()
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
        )
        if my_controller.is_done():
            current_state = "Done picking and placing"
        else:
            current_state = f"Phase: {my_controller.get_current_event()}"
        if current_state != previous_state:
            print(current_state)
            previous_state = current_state
        articulation_controller.apply_action(actions)

simulation_app.close()
