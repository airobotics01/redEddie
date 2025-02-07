# EULA (최종 사용자 라이선스 계약) 동의 설정
import os

os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES"

# Specify the GPU to use (e.g., GPU 0)
os.environ["CUDA_VISIBLE_DEVICES"] = "0"

CONFIG = {
    "renderer": "RayTracedLighting",
    "headless": False,
    "width": 1920,
    "height": 1080,
    "num_frames": 30,
}

# Initialize app FIRST
from isaacsim import SimulationApp

simulation_app = SimulationApp(CONFIG)

# Isaac Sim 환경 내 작업 시작
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.robots import Robot
from pxr import UsdPhysics
import omni.usd
import carb

# 물리 시뮬레이션 월드 생성
world = World()

# 기본 지면 추가
world.scene.add_default_ground_plane()

# FR3 로봇 추가
fr3_prim_path = "/FR3"
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
else:
    asset_path = assets_root_path + "/Isaac/Robots/Franka/FR3/fr3.usd"
    add_reference_to_stage(usd_path=asset_path, prim_path=fr3_prim_path)

# FR3 로봇 객체 생성
fr3_robot = world.scene.add(Robot(prim_path=fr3_prim_path, name="fr3"))

from pxr import UsdPhysics

stage = omni.usd.get_context().get_stage()
fr3_joint2_drive = UsdPhysics.DriveAPI.Get(
    stage.GetPrimAtPath("/FR3/fr3_link1/fr3_joint2"), "angular"
)
fr3_joint2_drive.GetStiffnessAttr().Set(0)

# 시뮬레이션 시작
world.reset()

from omni.isaac.dynamic_control import _dynamic_control
import numpy as np

dc = _dynamic_control.acquire_dynamic_control_interface()
# Note: getting the articulation has to happen after changing the drive stiffness
articulation = dc.get_articulation("/FR3")
dc.wake_up_articulation(articulation)
dof_ptr = dc.find_articulation_dof(articulation, "fr3_joint2")
dc.set_dof_velocity_target(dof_ptr, -0.2)


# Get information about the structure of the articulation
num_joints = dc.get_articulation_joint_count(articulation)
num_dofs = dc.get_articulation_dof_count(articulation)
num_bodies = dc.get_articulation_body_count(articulation)
print(num_joints, num_dofs, num_bodies)

# Get a specific degree of freedom on an articulation
dof_ptr = dc.find_articulation_dof(articulation, "fr3_joint2")
print(dof_ptr)

# 시뮬레이션 실행 (5초 동안)
import time

start_time = time.time()
while time.time() - start_time < 5:
    # Get complete state (position, velocity, effort)
    joint_state = dc.get_dof_state(dof_ptr, -1)
    print(
        f"Position: {joint_state.pos:.4f} rad, Velocity: {joint_state.vel:.4f} rad/s, Effort: {joint_state.effort:.4f} Nm"
    )

    # Step simulation
    world.step(render=True)

# 시뮬레이션 종료
simulation_app.close()
