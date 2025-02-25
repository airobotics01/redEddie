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
import omni.kit.viewport.utility
import carb


class InfoWindow(ui.Window):
    def __init__(self, title: str, **kwargs):
        super().__init__(title, width=300, height=100, **kwargs)
        self.frame.set_build_fn(self._build_fn)
        self.fps_label = None
        self.gpu_label = None
        self._update_sub = None

    def _build_fn(self):
        with ui.VStack():
            self.fps_label = ui.Label("FPS: N/A")

    def update_info(self):
        viewport = omni.kit.viewport.utility.get_active_viewport()
        fps = viewport.fps if viewport else "N/A"
        self.fps_label.text = f"FPS: {fps:.2f}"

    def start_update(self):
        self._update_sub = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(self._on_update, name="update_info_window")
        )

    def stop_update(self):
        if self._update_sub:
            self._update_sub.unsubscribe()
            self._update_sub = None

    def _on_update(self, e: carb.events.IEvent):
        self.update_info()


class HelloButtonWindow(ui.Window):
    def __init__(self, title: str, **kwargs):
        super().__init__(title, width=300, height=200, **kwargs)
        self.frame.set_build_fn(self._build_fn)
        self.count = 0

    def _build_fn(self):
        with ui.VStack():
            ui.Button("Say Hello", clicked_fn=self.say_hello)

    def say_hello(self):
        self.count += 1
        print(f"Hello, {self.count}")


hello_window = HelloButtonWindow("Hello Button Example")

info_window = InfoWindow("Viewport Info")
info_window.start_update()


simulation_app.update()

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
