from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})


from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.nucleus import get_assets_root_path

world = World()

assets_root_path = get_assets_root_path()

add_reference_to_stage(
    usd_path=f"{assets_root_path}/Isaac/Environments/Simple_Room/simple_room.usd",
    prim_path="/World/SimpleRoom",
)
add_reference_to_stage(
    usd_path=f"{assets_root_path}/Isaac/Robots/Franka/FR3/fr3.usd",
    prim_path="/World/FR3",
)

simulation_app.update()

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
