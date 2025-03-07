from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.stage import get_stage_units

from controllers.pick_place_controller import PickPlaceController
from tasks.pick_place import PickPlace

FINGER_LENGTH = 0.05  # 50mm in meters
MARGIN = 0.02

my_world = World(stage_units_in_meters=1.0)
cube_size = np.array([0.17, 0.0515, 0.185])  # np.array([0.0515, 0.0515, 0.0515])

if cube_size[2] > FINGER_LENGTH:
    hs_offset = cube_size[2] / 2 - FINGER_LENGTH + MARGIN

my_task = PickPlace(cube_size=cube_size)
my_world.add_task(my_task)
my_world.reset()
task_params = my_task.get_params()
my_fr3 = my_world.scene.get_object(task_params["robot_name"]["value"])
my_controller = PickPlaceController(
    name="pick_place_controller",
    gripper=my_fr3.gripper,
    robot_articulation=my_fr3,
    lift_offset=hs_offset,
)
articulation_controller = my_fr3.get_articulation_controller()

i = 0
reset_needed = False
previous_state = None
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
