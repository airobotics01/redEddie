# redEddie

사용 버전: Isaac Sim 4.5.0  

## 1. 환경 설정

- **아이작 심(Inaac Sim) 환경 설정**  
  - [x] Isaac Sim 설치 및 실행 확인  
  - [x] Isaac Sim 관련 기본 설정 및 Python 연동 환경 구축  
    [Isaac Sim 환경 설정](https://docs.google.com/presentation/d/1CxznysS31_eivuw4XQwRi4XAVka0ZGLmdPLkh7UrW18/edit?usp=sharing)
  - [x] Isaac Sim WorkFlow 동작 확인  
    [파이썬 워크플로우 슬라이드](https://docs.google.com/presentation/d/1tuupP8WfmBjFYPCMPFUajpIGdrajUykwrJfQsscG4X4/edit?usp=sharing)

## 2. 파이썬 스크립트와 API 활용

- **파이썬 스크립트와 API 활용**  
  [FR3 컨트롤 기초 슬라이드](https://docs.google.com/presentation/d/1ImJbjB4ewEsP1DvQoYI8wH1XchSMkLhhVbunT6sv5l8/edit?usp=sharing)  
  [FR3_pick_place 슬라이드](https://docs.google.com/presentation/d/1Utw_5IjKaYB-rfhFC_A1U_ccb3WO2LkoUuoIVhKKxfQ/edit?usp=sharing)  
  
  - [x] 파이썬 스크립트를 활용한 로봇 동작 튜토리얼  
    [franka_gripper.py](https://github.com/airobotics01/redEddie/blob/main/FR3/franka_gripper.py), [follow_target_with_ik.py](https://github.com/airobotics01/redEddie/blob/main/FR3/follow_target_with_ik.py), [follow_target_with_rmpflow.py](https://github.com/airobotics01/redEddie/blob/main/FR3/follow_target_with_rmpflow.py)
  - [x] Franka Research 3 활용 [Pick Place Task](https://docs.isaacsim.omniverse.nvidia.com/latest/core_api_tutorials/tutorial_core_adding_manipulator.html#) 생성  
    [FR3_pick_place.py](https://github.com/airobotics01/redEddie/blob/main/FR3_pick_place.py) 또는 [pick_place.py](https://github.com/airobotics01/redEddie/blob/main/FR3/pick_place.py)
  - [x] Franka Research 3 활용 Stacking Task 생성  
    [FR3_stacking.py](https://github.com/airobotics01/redEddie/blob/4.5.0/FR3_stacking.py), [stacking.py](https://github.com/airobotics01/redEddie/blob/main/FR3/stacking.py)
  - [ ] [Stacking Decider](https://docs.isaacsim.omniverse.nvidia.com/latest/cortex_tutorials/tutorial_cortex_4_franka_block_stacking.html#)
  - [x] [Multiple Tasks](https://docs.isaacsim.omniverse.nvidia.com/latest/core_api_tutorials/tutorial_core_multiple_tasks.html#)  
        [multitpe_tasks.py](https://github.com/airobotics01/redEddie/blob/main/FR3/multiple_tasks.py)
  - Custom task 생성
    - [x] Franka Research 3를 활용하여 원통형 물체에 대해 PickPlace  
      [pick_place_cylinder.py](https://github.com/airobotics01/redEddie/blob/main/FR3/pick_place_cylinder.py)  
    - [ ] 여러 종류의 물체를 생성하고 PickPlace


## 기타

- [ ] Isaac Manipulator  
  - FoundationPose와 DOPE를 이용해 UR10 로봇 팔로 토마토캔을 잡는 작업을 시도했으나, 실행이 원활하지 않음.
