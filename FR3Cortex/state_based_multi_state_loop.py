from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import time
import random
import numpy as np
from isaacsim.core.api.objects import VisualSphere
from isaacsim.cortex.framework.cortex_world import CortexWorld
from isaacsim.cortex.framework.df import (
    DfNetwork,
    DfState,
    DfStateMachineDecider,
    DfStateSequence,
)
from isaacsim.cortex.framework.dfb import DfRobotApiContext
from robot import add_fr3_to_stage


# 로봇 상태를 저장하고 공유하는 Context 클래스
class RobotStateContext(DfRobotApiContext):
    def __init__(self, robot):
        super().__init__(robot)
        self.robot = robot
        self.reset()

        # 모니터링 함수 등록
        self.add_monitors(
            [
                RobotStateContext.monitor_target_reached,
                RobotStateContext.monitor_diagnostics,
            ]
        )

    def reset(self):
        # 상태와 관련된 데이터 초기화
        self.random_position = None
        self.home_position = np.array([0.4, 0.0, 0.3])  # 홈 포지션 좌표
        self.is_at_home = False
        self.is_at_random_position = False
        self.last_state_change_time = time.time()
        self.gripper_state = "closed"  # 그리퍼 초기 상태
        self.diagnostics_message = ""

    def monitor_target_reached(self):
        """현재 로봇 위치가 목표 위치에 도달했는지 확인"""
        current_position = self.robot.arm.get_fk_p()

        # 홈 포지션 도달 여부 확인
        if np.linalg.norm(current_position - self.home_position) < 0.03:
            self.is_at_home = True
        else:
            self.is_at_home = False

        # 랜덤 포지션 도달 여부 확인
        if (
            self.random_position is not None
            and np.linalg.norm(current_position - self.random_position) < 0.03
        ):
            self.is_at_random_position = True
        else:
            self.is_at_random_position = False

    def monitor_diagnostics(self):
        """현재 상태를 진단 메시지로 출력"""
        current_pos = self.robot.arm.get_fk_p()

        self.diagnostics_message = f"""
        현재 로봇 상태:
        위치: {current_pos}
        홈 포지션: {self.home_position}
        랜덤 포지션: {self.random_position}
        홈 위치 도달: {self.is_at_home}
        랜덤 위치 도달: {self.is_at_random_position}
        그리퍼 상태: {self.gripper_state}
        마지막 상태 변경 후 경과 시간: {time.time() - self.last_state_change_time:.2f}초
        """

        print(self.diagnostics_message)


# 1. 홈 포지션으로 이동하는 상태
class HomePositionState(DfState):
    def __init__(self, min_time_in_state=2.0):
        super().__init__()
        self.min_time_in_state = min_time_in_state  # 이 상태에 최소한 머무는 시간

    def enter(self):
        """상태 진입 시 호출되는 함수"""
        print("홈 포지션 상태로 진입합니다.")
        self.context.last_state_change_time = time.time()

        # 타겟 마커가 있으면 홈 포지션으로 이동
        if hasattr(self.context.robot, "target_marker"):
            self.context.robot.target_marker.set_world_pose(
                self.context.home_position, np.array([0, 0, 0, 1])
            )

        # 로봇에게 홈 포지션으로 이동하라는 명령 전송
        self.context.robot.arm.send_end_effector(
            target_position=self.context.home_position
        )

    def step(self):
        """매 시뮬레이션 스텝마다 호출되는 함수"""
        current_time = time.time()

        # 홈 위치에 도달했고, 최소 상태 시간이 지났는지 확인
        if (
            self.context.is_at_home
            and current_time - self.context.last_state_change_time
            >= self.min_time_in_state
        ):
            return None  # 다음 상태로 전환

        # 아직 홈 위치에 도달하지 않았거나 최소 시간이 지나지 않았으면 계속 이 상태 유지
        return self


# 2. 랜덤 위치 생성 상태
class GenerateRandomPositionState(DfState):
    def __init__(self):
        super().__init__()

    def enter(self):
        """상태 진입 시 호출되는 함수"""
        print("랜덤 위치 생성 상태로 진입합니다.")
        self.context.last_state_change_time = time.time()

        # 유효한 랜덤 위치 생성
        self.generate_random_position()

        # 타겟 마커가 있으면 랜덤 포지션으로 이동
        if hasattr(self.context.robot, "target_marker"):
            self.context.robot.target_marker.set_world_pose(
                self.context.random_position, np.array([0, 0, 0, 1])
            )

    def generate_random_position(self):
        """작업 공간 내의 유효한 랜덤 위치 생성"""
        # 로봇 작업 영역 내의 랜덤 위치 생성
        x = random.uniform(0.3, 0.7)  # 전방 거리
        y = random.uniform(-0.3, 0.3)  # 좌우 방향
        z = random.uniform(0.2, 0.6)  # 높이

        # Context에 저장
        self.context.random_position = np.array([x, y, z])
        print(f"새로운 랜덤 위치가 생성되었습니다: {self.context.random_position}")

    def step(self):
        """매 시뮬레이션 스텝마다 호출되는 함수"""
        # 랜덤 위치 생성은 즉시 완료되므로 바로 다음 상태로 전환
        return None


# 3. 랜덤 위치로 이동하는 상태
class MoveToRandomPositionState(DfState):
    def __init__(self, min_time_in_state=3.0):
        super().__init__()
        self.min_time_in_state = min_time_in_state

    def enter(self):
        """상태 진입 시 호출되는 함수"""
        print("랜덤 위치로 이동하는 상태로 진입합니다.")
        self.context.last_state_change_time = time.time()

        # 로봇에게 랜덤 위치로 이동하라는 명령 전송
        self.context.robot.arm.send_end_effector(
            target_position=self.context.random_position
        )

    def step(self):
        """매 시뮬레이션 스텝마다 호출되는 함수"""
        current_time = time.time()

        # 랜덤 위치에 도달했고, 최소 상태 시간이 지났는지 확인
        if (
            self.context.is_at_random_position
            and current_time - self.context.last_state_change_time
            >= self.min_time_in_state
        ):
            return None  # 다음 상태로 전환

        # 아직 랜덤 위치에 도달하지 않았거나 최소 시간이 지나지 않았으면 계속 이 상태 유지
        return self


# 4. 그리퍼 액션 상태
class GripperActionState(DfState):
    def __init__(self):
        super().__init__()

    def enter(self):
        """상태 진입 시 호출되는 함수"""
        print("그리퍼 액션 상태로 진입합니다.")
        self.context.last_state_change_time = time.time()

        # 그리퍼 토글(열기/닫기)
        if self.context.gripper_state == "closed":
            self.context.robot.gripper.open(speed=0.5)
            self.context.gripper_state = "open"
            print("그리퍼를 엽니다.")
        else:
            self.context.robot.gripper.close(speed=0.5)
            self.context.gripper_state = "closed"
            print("그리퍼를 닫습니다.")

    def step(self):
        """매 시뮬레이션 스텝마다 호출되는 함수"""
        current_time = time.time()

        # 그리퍼 액션 완료 시간 (1초 후)
        if current_time - self.context.last_state_change_time >= 1.0:
            return None  # 다음 상태로 전환

        return self


def main():
    # 시뮬레이션 월드 생성
    world = CortexWorld()
    world.scene.add_default_ground_plane()

    # FR3 로봇 추가
    robot = world.add_robot(add_fr3_to_stage(name="my_fr3", prim_path="/World/fr3"))

    # 타겟 위치를 시각화할 마커 추가
    robot.target_marker = world.scene.add(
        VisualSphere(
            name="target_marker",
            prim_path="/World/TargetMarker",
            radius=0.02,
            color=np.array([1.0, 0.5, 0.0]),  # 주황색
        )
    )

    # 상태 시퀀스 생성 - 반복적으로 실행될 상태들의 순서 지정
    state_sequence = DfStateSequence(
        [
            HomePositionState(min_time_in_state=2.0),
            GenerateRandomPositionState(),
            MoveToRandomPositionState(min_time_in_state=3.0),
            GripperActionState(),
        ],
        loop=True,  # 상태 시퀀스를 무한 반복
    )

    # 로봇 상태 컨텍스트 생성
    context = RobotStateContext(robot)

    # 상태 머신 결정기 네트워크 생성
    decider_network = DfNetwork(DfStateMachineDecider(state_sequence), context=context)

    # 결정기 네트워크를 월드에 추가
    world.add_decider_network(decider_network)

    # 시뮬레이션 실행
    print("시뮬레이션을 시작합니다. 로봇이 홈 포지션과 랜덤 포지션 사이를 이동합니다.")
    print("종료하려면 Ctrl+C를 누르세요.")

    world.reset()
    world.run(simulation_app)
    simulation_app.close()


if __name__ == "__main__":
    main()
