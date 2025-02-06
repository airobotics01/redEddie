# EULA (최종 사용자 라이선스 계약) 동의 설정
import os

os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES"

CONFIG = {
    "renderer": "RayTracedLighting",
    "headless": False,
    "width": 1980,
    "height": 1080,
    "num_frames": 24,
}

# Initialize app FIRST
from isaacsim import SimulationApp

simulation_app = SimulationApp(CONFIG)

# Isaac Sim 환경 내 작업 시작
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path

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

# 시뮬레이션 시작
world.reset()

# 시뮬레이션 실행 (5초 동안)
import time

start_time = time.time()
while time.time() - start_time < 5:
    world.step(render=True)

# 시뮬레이션 종료
simulation_app.close()
