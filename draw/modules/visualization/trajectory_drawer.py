import random
from isaacsim.util.debug_draw import _debug_draw


class TrajectoryDrawer:
    """궤적을 시각화하기 위한 Debug 클래스"""

    def __init__(self):
        self.draw = None
        self.point_list = []
        self.colors = (
            random.uniform(0, 1),
            random.uniform(0, 1),
            random.uniform(0, 1),
            1,
        )  # RGBa

    def initialize(self):
        """Debug Draw 인터페이스 초기화"""
        self.draw = _debug_draw.acquire_debug_draw_interface()

    def update_drawing(self, position, draw_offset=[0, 0, 0]):
        """
        하트 궤적을 그리기 위한 포인트 업데이트 및 시각화

        Args:
            position: Task에서 계산된 현재 위치
            draw_offset: 그리기 위치 오프셋 (선택 사항)
        """
        # 포인트 리스트에 현재 위치 추가 (주기적으로)
        self.point_list.append(tuple(position + draw_offset))

        # 가끔씩 라인 정리
        if len(self.point_list) % 10 == 0:
            self.draw.clear_lines()

        # 경로 그리기
        if len(self.point_list) != 0:
            self.draw.draw_lines_spline(self.point_list, self.colors, 5, False)

        # 버퍼 크기 관리
        if len(self.point_list) > 70:
            del self.point_list[0]

    def reset_drawing(self):
        """하트 궤적 초기화"""
        self.point_list = []
