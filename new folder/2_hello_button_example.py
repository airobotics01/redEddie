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


import omni.ui as ui


class HelloButtonWindow(ui.Window):
    def __init__(self, title: str, **kwargs):
        super().__init__(title, width=300, height=200, **kwargs)
        self.frame.set_build_fn(self._build_fn)

    def _build_fn(self):
        with ui.VStack():
            ui.Button("Say Hello", clicked_fn=self.say_hello)

    def say_hello(self):
        print("Hello")


# 윈도우 생성 및 표시
hello_window = HelloButtonWindow("Hello Button Example")
# hello_window.show()


simulation_app.update()

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
