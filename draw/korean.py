import os
import json
import numpy as np

script_path = os.path.abspath(__file__)
json_path = os.path.dirname(script_path)
json_path += "/asset/korean.json"
with open(json_path, encoding="utf-8") as f:
    data = json.load(f)
    character_path = data["characters"]

# 전역 상태 변수 추가
current_state = 0
last_stroke = -1  # 마지막으로 처리한 획 번호 추적


def convert_coordinate(approach_point):
    """로봇 좌표계로 변환하는 함수
    Args:
        approach_point (np.array): 변환할 3D 좌표점 [x, y, z]
    Returns:
        np.array: 변환된 좌표 [x', y', z']
    """
    x, y, z = approach_point
    return np.array([-z, y, x])


def find_paths_by_name(character_name):
    for character in data["characters"]:
        if character["name"] == character_name:
            return character["path"]
    return None


def get_coordinate(i):
    return [
        (i["start"][0], i["start"][1], i["start"][2]),
        (i["end"][0], i["end"][1], i["end"][2]),
    ]


def generate_korean_character(
    tick,
    character_name,
    current_stroke=0,
    draw_scale=1.0,
    ee_pos=None,
    original_position=None,
):
    """한글 문자 궤적 생성
    Args:
        tick (int): 현재 틱
        character_name (str): 그릴 한글 문자
        current_stroke (int): 현재 획 번호
        draw_scale (float): 그리기 스케일
        ee_pos (np.array): 현재 엔드 이펙터 위치
        original_position (np.array): 원점 위치 [x, y, z]
    Returns:
        tuple: (trajectory, is_stroke_complete)
            - trajectory: 다음 위치 좌표 (획이 완료되면 None)
            - is_stroke_complete: 현재 획이 완료되었는지 여부
    """
    global current_state, last_stroke

    # korean.json에서 문자 경로 가져오기
    character_path = find_paths_by_name(character_name)
    if not character_path or current_stroke >= len(character_path):
        current_state = 0
        last_stroke = -1
        return None, True

    # 새로운 획으로 넘어갈 때 상태 초기화
    if current_stroke != last_stroke:
        current_state = 0
        last_stroke = current_stroke

    # 현재 획의 시작점과 끝점
    current_path = character_path[current_stroke]
    start_point = np.array(current_path["start"])
    end_point = np.array(current_path["end"])

    # 현재 엔드 이펙터 위치 가져오기
    ee_pos = ee_pos
    if ee_pos is None:
        return None, False

    # 시작점으로부터의 거리 계산
    _start_point = (
        convert_coordinate(start_point) + original_position
    )  # 로봇 좌표계로 변환하고 원점 더하기
    _end_point = (
        convert_coordinate(end_point) + original_position
    )  # 로봇 좌표계로 변환하고 원점 더하기
    distance_to_start = np.linalg.norm(ee_pos - _start_point)
    distance_to_end = np.linalg.norm(ee_pos - _end_point)

    if current_state == 0:
        print(f"다가가기: {distance_to_start}")
        approach_point = start_point.copy()
        approach_point[0] += 0.05  # x값에 5cm 더하기
        if distance_to_start <= 0.05:
            current_state = 1
        return convert_coordinate(approach_point) + original_position, False

    elif current_state == 1:
        print(f"그리기 준비: {distance_to_start}")
        if distance_to_start <= 0.01:
            current_state = 2
        return convert_coordinate(start_point) + original_position, False

    elif current_state == 2:
        # 시작점과 끝점 사이의 진행률 계산
        total_distance = np.linalg.norm(_end_point - _start_point)
        current_distance = np.linalg.norm(ee_pos - _start_point)
        # draw_scale을 사용하여 진행률 계산 속도 조절
        progress = min(1.0, (current_distance / total_distance) * draw_scale)
        print(f"그리기: {progress*100} %")

        # 시작점과 끝점 사이를 보간
        approach_point = start_point + (end_point - start_point) * progress
        if distance_to_end <= 0.02:
            current_state = 3
            return convert_coordinate(approach_point) + original_position, True
        return convert_coordinate(approach_point) + original_position, False

    # elif current_state == 3:
    #     # 질질 끌지 않도록. 획이 확실하게 분리될때만 동작해야함.
    #     # print(f"멀어지기: {distance_to_end}")
    #     return None, False
