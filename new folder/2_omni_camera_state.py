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

from isaacsim.core.utils.viewports import set_camera_view
from omni.kit.viewport.utility.camera_state import ViewportCameraState

# 카메라의 위치와 타겟 설정
eye_position = [0.0, 2.0, 0.5]  # 카메라의 위치 (x, y, z)
target_position = [0.0, 0.0, 0.5]  # 카메라가 바라보는 대상의 위치 (x, y, z)

# 기본 Viewport 카메라 경로
camera_prim_path = "/OmniverseKit_Persp"

# 카메라 뷰 설정
set_camera_view(
    eye=eye_position, target=target_position, camera_prim_path=camera_prim_path
)

from omni.usd import get_context
from pxr import UsdGeom, Gf
import numpy as np


last_camera_state = None


def print_camera_state(camera_prim_path):
    def rotation_matrix_to_euler(R):
        r11, r12, r13 = R[0]
        r21, r22, r23 = R[1]
        r31, r32, r33 = R[2]

        # Pitch (y-axis rotation)
        pitch = np.arcsin(-r31)

        # Roll (x-axis rotation)
        roll = np.arctan2(r32, r33)

        # Yaw (z-axis rotation)
        yaw = np.arctan2(r21, r11)

        return np.array([roll, pitch, yaw])

    def format_vec3d(vec, precision=2):
        return (
            f"({vec[0]:.{precision}f}, {vec[1]:.{precision}f}, {vec[2]:.{precision}f})"
        )

    def format_vec4d(vec, precision=2):
        if isinstance(vec, Gf.Quatd):
            return f"({vec.GetReal():.{precision}f}, {vec.GetImaginary()[0]:.{precision}f}, {vec.GetImaginary()[1]:.{precision}f}, {vec.GetImaginary()[2]:.{precision}f})"
        else:
            return f"({vec[0]:.{precision}f}, {vec[1]:.{precision}f}, {vec[2]:.{precision}f}, {vec[3]:.{precision}f})"

    global last_camera_state

    stage = get_context().get_stage()
    camera_prim = stage.GetPrimAtPath(camera_prim_path)

    if camera_prim.IsValid():
        xformable = UsdGeom.Xformable(camera_prim)
        transform = xformable.ComputeLocalToWorldTransform(0.0)
        position = transform.ExtractTranslation()
        rotation = transform.ExtractRotationQuat()
        rpy = rotation_matrix_to_euler(transform.ExtractRotationMatrix())

        current_state = (position, rotation)

        if current_state != last_camera_state:
            print(f"Camera Position: {format_vec3d(position)}")
            print(f"Camera Orientation (Quaternion): {format_vec4d(rotation)}")
            print(f"Camera Orientation (RPY): {format_vec3d(np.degrees(rpy))}")
            print()
            last_camera_state = current_state
    else:
        print(f"Camera Prim at path {camera_prim_path} is not valid.")
        print()


simulation_app.update()

while simulation_app.is_running():
    simulation_app.update()

    print_camera_state(camera_prim_path)

simulation_app.close()
