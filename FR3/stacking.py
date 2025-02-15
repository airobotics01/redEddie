from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api import World

from controllers.stacking_controller import StackingController
from tasks.stacking import Stacking

my_world = World(stage_units_in_meters=1.0)

cube_size = np.array([0.0515, 0.0515, 0.0515])
cube_positions = np.array(
    [
        [0.5, -0.3, 0.3],
        [0.5, 0.0, 0.3],
        [0.5, 0.3, 0.3],
    ]
)
cube_positions[:, 2] = cube_size[2] / 2
my_task = Stacking(
    cube_initial_positions=cube_positions,
    cube_size=cube_size,
)
my_world.add_task(my_task)
my_world.reset()
robot_name = my_task.get_params()["robot_name"]["value"]
my_fr3 = my_world.scene.get_object(robot_name)
my_controller = StackingController(
    name="stacking_controller",
    gripper=my_fr3.gripper,
    robot_articulation=my_fr3,
    picking_order_cube_names=my_task.get_cube_names(),  # ["cube", "cube_1", "cube_2"]
    robot_observation_name=robot_name,
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
        actions = my_controller.forward(observations=observations)
        articulation_controller.apply_action(actions)
        if my_controller.is_done():
            current_state = "Done stacking"
        else:
            current_state = f"Stacking {my_controller.get_current_event()}"
        if current_state != previous_state:
            print(current_state)
            previous_state = current_state

simulation_app.close()

# TODO hs_offset을 각각의 task마다 줄 수 있는 방법을 생각해보자.
