from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False, "num_frames": 30})

from isaacsim.cortex.framework.cortex_world import CortexWorld
from isaacsim.cortex.framework.df import DfNetwork, DfDecider, DfDecision
from isaacsim.cortex.framework.dfb import DfRobotApiContext
from robot import add_fr3_to_stage
import time


# 1. 확장된 컨텍스트 정의
class RobotContext(DfRobotApiContext):
    def __init__(self, robot):
        super().__init__(robot)
        self.simulation_start_time = time.time()
        self.reset()
        self.add_monitors([RobotContext.monitor_state])
        self.next_action_time = 5.0

    def reset(self):
        self.iteration = 0
        self.should_switch_state = False

    def monitor_state(self):
        # 카운터 증가
        self.iteration += 1
        # print(f"Monitor state called: iteration {self.iteration}")

        current_time = time.time() - self.simulation_start_time
        if current_time > self.next_action_time:
            self.next_action_time = current_time + 5.0
            self.should_switch_state = True
            print(
                f"상태 전환 신호 발생! 현재 시간: {current_time:.2f}, 다음 전환 시간: {self.next_action_time:.2f}"
            )


# 2. 메인 코디네이터 결정자 정의 (루트 노드)
class MainCoordinatorDecider(DfDecider):
    def __init__(self, state_a, state_b):
        super().__init__()
        # 두 개의 자식 결정자 추가
        self.add_child("state_a", state_a)
        self.add_child("state_b", state_b)
        self.current_state = "state_a"  # 초기 상태 설정

    def enter(self):
        print("메인 코디네이터에 진입했습니다")

    def decide(self):
        # 상태 전환 조건을 확인
        if self.context.should_switch_state:
            # 상태 전환 신호 초기화
            self.context.should_switch_state = False

            # 상태 전환
            if self.current_state == "state_a":
                self.current_state = "state_b"
                print(f"상태 A -> 상태 B로 전환 (iteration: {self.context.iteration})")
            else:
                self.current_state = "state_a"
                print(f"상태 B -> 상태 A로 전환 (iteration: {self.context.iteration})")

        # 현재 상태에 해당하는 자식 결정자 반환
        if self.current_state == "state_a":
            return DfDecision("state_a")
        else:
            return DfDecision("state_b")

    def exit(self):
        print("메인 코디네이터를 종료합니다")


# 3. 첫 번째 상태 결정자 정의
class StateADecider(DfDecider):
    def __init__(self):
        super().__init__()

    def enter(self):
        print("상태 A에 진입했습니다")
        self._start_time = time.time()
        self._last_print_time = self._start_time

    def decide(self):
        # 상태 A에서의 로봇 동작 로직
        # 1초마다 반복되는 로그 출력
        current_time = time.time()

        if current_time - self._last_print_time >= 1.0:
            print(f"상태 A 실행 중: iteration {self.context.iteration}")
            self._last_print_time += 1.0

        # 자식 결정자가 없으므로 None 반환
        return None

    def exit(self):
        print("상태 A를 종료합니다")


# 4. 두 번째 상태 결정자 정의
class StateBDecider(DfDecider):
    def __init__(self):
        super().__init__()

    def enter(self):
        print("상태 B에 진입했습니다")
        self._start_time = time.time()
        self._last_print_time = self._start_time

    def decide(self):
        # 상태 B에서의 로봇 동작 로직
        # 1초마다 반복되는 로그 출력
        current_time = time.time()

        if current_time - self._last_print_time >= 1.0:
            print(f"상태 B 실행 중: iteration {self.context.iteration}")
            self._last_print_time += 1.0

        # 자식 결정자가 없으므로 None 반환
        return None

    def exit(self):
        print("상태 B를 종료합니다")


# 5. 시뮬레이션 환경 구성
world = CortexWorld()
robot = world.add_robot(add_fr3_to_stage(name="fr3", prim_path="/World/fr3"))
world.scene.add_default_ground_plane()

# 6. 결정자 네트워크 구성
state_a = StateADecider()
state_b = StateBDecider()
root_decider = MainCoordinatorDecider(state_a, state_b)
context = RobotContext(robot)
decider_network = DfNetwork(root_decider, context=context)
world.add_decider_network(decider_network)

# 7. 시뮬레이션 실행
world.reset()
print("시뮬레이션을 시작합니다...")
world.run(simulation_app)
print("시뮬레이션이 완료되었습니다.")
simulation_app.close()
