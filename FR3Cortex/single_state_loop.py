from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import time
import numpy as np
from isaacsim.core.api.objects import VisualSphere
from isaacsim.cortex.framework.cortex_world import CortexWorld
from isaacsim.cortex.framework.df import (
    DfNetwork,
    DfState,
    DfStateMachineDecider,
    DfStateSequence,
)
from isaacsim.cortex.framework.dfb import DfBasicContext

# 기존 robot.py에서 FR3 로봇 헬퍼 함수 가져오기
from robot import add_fr3_to_stage


class RandomPositionFollowerState(DfState):
    """로봇이 무작위로 변하는 위치를 따라가게 하는 상태.

    목표 위치는 몇 초마다 로봇 작업 공간 내의
    새로운 무작위 위치로 변경됩니다.
    """

    def __init__(self, update_interval=3.0):
        super().__init__()
        # 업데이트 간격 설정(초 단위)
        self.update_interval = update_interval
        # 상태 생성 시간 저장
        self.construction_time = time.time()
        # 대상 시각 마커 초기화
        self.target_marker = None

    def sample_random_position(self):
        """로봇 작업 공간 내에서 무작위 위치 생성"""
        # 작업 공간 내 무작위 위치
        x = np.random.uniform(0.3, 0.7)  # 전방 거리
        y = np.random.uniform(-0.3, 0.3)  # 좌우 거리
        z = np.random.uniform(0.2, 0.6)  # 높이
        return np.array([x, y, z])

    def enter(self):
        """이 상태로 진입할 때 호출"""
        print(
            f"[{time.time() - self.construction_time:.2f}] Entering RandomPositionFollowerState"
        )

        # 다음 목표 시간 초기화
        self.next_target_time = time.time() + self.update_interval

        # 첫 번째 목표 위치 샘플링
        self.target_p = self.sample_random_position()

        # 시각적 목표 마커 생성 또는 업데이트
        if self.target_marker is None and hasattr(self.context.robot, "target_marker"):
            self.target_marker = self.context.robot.target_marker

        # 목표 마커 위치 업데이트
        if self.target_marker:
            self.target_marker.set_world_pose(self.target_p, np.array([0, 0, 0, 1]))

        # 그리퍼 상태 전환(열린 상태면 닫고, 닫힌 상태면 열기)
        gripper = self.context.robot.gripper
        if gripper.get_width() > 0.04:  # 열린 상태면
            gripper.close(speed=0.5)
            print(f"[{time.time() - self.construction_time:.2f}] Closing gripper")
        else:  # 닫힌 상태면
            gripper.open(speed=0.5)
            print(f"[{time.time() - self.construction_time:.2f}] Opening gripper")

    def step(self):
        """이 상태가 활성 상태인 동안 반복적으로 호출"""
        current_time = time.time()

        # 목표 위치 업데이트 시간인지 확인
        if current_time >= self.next_target_time:
            # 새 무작위 위치 샘플링
            self.target_p = self.sample_random_position()
            print(
                f"[{current_time - self.construction_time:.2f}] New target: {self.target_p}"
            )

            # 시각적 마커 업데이트
            if self.target_marker:
                self.target_marker.set_world_pose(self.target_p, np.array([0, 0, 0, 1]))

            # 다음 업데이트 시간 설정
            self.next_target_time = current_time + self.update_interval

            # 그리퍼 상태 전환
            gripper = self.context.robot.gripper
            if gripper.get_width() > 0.04:  # 열린 상태면
                gripper.close(speed=0.5)
                print(f"[{current_time - self.construction_time:.2f}] Closing gripper")
            else:  # 닫힌 상태면
                gripper.open(speed=0.5)
                print(f"[{current_time - self.construction_time:.2f}] Opening gripper")

        # 로봇을 현재 목표 위치로 이동
        self.context.robot.arm.send_end_effector(target_position=self.target_p)

        # 이 상태를 활성 상태로 유지하기 위해 self 반환
        return self


def main():
    # 시뮬레이션 월드 생성
    world = CortexWorld()

    # 바닥면 추가
    world.scene.add_default_ground_plane()

    # FR3 로봇 추가
    robot = world.add_robot(add_fr3_to_stage(name="my_fr3", prim_path="/World/fr3"))

    # 목표 위치를 나타내는 시각적 구체 추가
    robot.target_marker = world.scene.add(
        VisualSphere(
            name="target_marker",
            prim_path="/World/TargetMarker",
            radius=0.02,
            color=np.array([1.0, 0.5, 0.0]),  # 주황색
        )
    )

    # 무작위 위치 추적 상태로 상태 머신 생성
    # update_interval 매개변수는 목표 위치가 변경되는 빈도 제어(초 단위)
    follower_state = RandomPositionFollowerState(update_interval=3.0)

    # 상태와 함께 결정 프레임워크 네트워크 생성
    decider_network = DfNetwork(
        DfStateMachineDecider(DfStateSequence([follower_state], loop=True)),
        context=DfBasicContext(robot),
    )

    # 월드에 결정자 네트워크 추가
    world.add_decider_network(decider_network)

    # 시뮬레이션 실행
    print("Starting simulation - Robot will follow random positions")
    print("The target position will change every 3 seconds")
    print("Press Ctrl+C to exit")

    world.reset()
    world.run(simulation_app)
    simulation_app.close()


if __name__ == "__main__":
    main()
