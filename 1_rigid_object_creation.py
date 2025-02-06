# EULA (최종 사용자 라이선스 계약) 동의 설정
import os
os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES"

# Isaac Sim 실행 설정
CONFIG = {
    "renderer": "RayTracedLighting",  # 레이 트레이싱 렌더링 사용
    "headless": False,                # GUI 모드로 실행 (True면 헤드리스 모드)
    "width": 1980,                    # 뷰포트 너비
    "height": 1080,                   # 뷰포트 높이
    "num_frames": 24                  # 프레임 수 (초당 프레임)
}

# Isaac Sim 애플리케이션 초기화
from isaacsim import SimulationApp
simulation_app = SimulationApp(CONFIG)

# Isaac Sim 환경 내 작업 시작
# 주의: omni.isaac.core 모듈은 Isaac Sim이 실행된 후에만 import 가능

import numpy as np
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

# 물리 시뮬레이션 월드 생성
world = World()
world.scene.add_default_ground_plane()  # 기본 지면 추가

# 동적 큐브 객체 생성
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",      # 큐브의 고유 경로
        position=[0, 0, 1.0],         # 초기 위치 (x, y, z)
        scale=[0.5, 0.5, 0.5],        # 크기 (너비, 길이, 높이)
        color=np.array([.2,.3,0.]),   # 색상 (RGB)
    )
)

# 현재 뷰포트 해상도 확인
from omni.kit.viewport.utility import get_active_viewport
viewport = get_active_viewport()
resolution = viewport.get_texture_resolution()  # (너비, 높이) 반환
print("Current viewport resolution:", resolution)

# 물리 시뮬레이션 초기화
world.reset()

# 메인 시뮬레이션 루프
while simulation_app.is_running():
    # 물리 시뮬레이션 단계 실행 및 렌더링
    world.step(render=True)
    
    # (선택사항) 객체 속성 접근
    print("Cube position:", cube.get_world_pose()[0])  # 큐브의 현재 위치 출력

# 시뮬레이션 종료 및 정리
simulation_app.close()
