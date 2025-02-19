# redEddie

**사용 버전**: Isaac Sim 4.5.0  

## 1. 환경 설정

- **아이작 심(Isaac Sim) 환경 설정**  
  - [x] Isaac Sim 설치 및 실행 확인  
  - [x] Isaac Sim 관련 기본 설정 및 Python 연동 환경 구축  
    [🗣️ Isaac Sim 환경 설정](https://docs.google.com/presentation/d/1CxznysS31_eivuw4XQwRi4XAVka0ZGLmdPLkh7UrW18/edit?usp=sharing)
  - [x] Isaac Sim WorkFlow 동작 확인  
    [🗣️ 파이썬 워크플로우 슬라이드](https://docs.google.com/presentation/d/1tuupP8WfmBjFYPCMPFUajpIGdrajUykwrJfQsscG4X4/edit?usp=sharing)

## 2. 파이썬 스크립트와 API 활용 (isaacsim.robot.manipulators)

- **파이썬 스크립트와 API 활용**  
  - [🗣️ FR3 컨트롤 기초 슬라이드](https://docs.google.com/presentation/d/1ImJbjB4ewEsP1DvQoYI8wH1XchSMkLhhVbunT6sv5l8/edit?usp=sharing)  
  - [🗣️ FR3_pick_place 슬라이드](https://docs.google.com/presentation/d/1Utw_5IjKaYB-rfhFC_A1U_ccb3WO2LkoUuoIVhKKxfQ/edit?usp=sharing)  
  *원래 코드가 잘 안 되어서 클래스 별로 파일을 나누고 FR3폴더에 정리함*

  - [x] 파이썬 스크립트를 활용한 로봇 동작 튜토리얼  
    - [🐍📝 franka_gripper.py](https://github.com/airobotics01/redEddie/blob/main/FR3/franka_gripper.py)  
    - [🐍📝 follow_target_with_ik.py](https://github.com/airobotics01/redEddie/blob/main/FR3/follow_target_with_ik.py)  
    - [🐍📝 follow_target_with_rmpflow.py](https://github.com/airobotics01/redEddie/blob/main/FR3/follow_target_with_rmpflow.py)  
    - [🎞️ 실습](https://youtu.be/QxAM6UtmvcU), [🎞️ 실습 2](https://youtu.be/_Yo0rGRb5ws), [🎞️ 실습 3](https://youtu.be/zt9luAetkK8)  

  - [x] Franka Research 3 활용  
    - **Pick Place Task** 생성  
      - [🐍📝FR3_pick_place.py](https://github.com/airobotics01/redEddie/blob/main/FR3/pick_place.py)  
      - [🎞️ 실습](https://youtu.be/a2e3Q8sN_EA)
      - [📄 관련 문서 (🏠︎ > Python Scripting > Core API Tutorial Series > Adding a Manipulator Robot)](https://docs.isaacsim.omniverse.nvidia.com/latest/core_api_tutorials/tutorial_core_adding_manipulator.html#)  
    - **Stacking Task** 생성  
      - [🐍📝FR3_stacking.py](https://github.com/airobotics01/redEddie/blob/4.5.0/FR3/staking.py)  
      - [🎞️ 실습](https://youtu.be/7UR9dSv3HA4)    

  - [x] **Multiple Tasks**  
    - [🐍📝multiple_tasks.py](https://github.com/airobotics01/redEddie/blob/main/FR3/multiple_tasks.py)  
    - [📄 관련 문서 (🏠︎ > Python Scripting > Core API Tutorial Series > Multiple Tasks)](https://docs.isaacsim.omniverse.nvidia.com/latest/core_api_tutorials/tutorial_core_multiple_tasks.html#)  

  - **Custom Task 생성**  
    - [x] Franka Research 3를 활용하여 원통형 물체에 대해 PickPlace  
      - [🐍📝pick_place_cylinder.py](https://github.com/airobotics01/redEddie/blob/main/FR3/pick_place_cylinder.py)  
      - [🎞️ 실습](https://youtu.be/Pc4mxiSyUh4)  
    - [ ] 여러 종류의 물체를 생성하고 PickPlace 작업 구현  

## 3. isaacsim.cortex.framework

Isaac Sim에서 Cortex는 협업 로봇 시스템을 위한 의사 결정 프레임워크입니다. Isaac Sim의 로보틱스 도구들을 하나의 일관된 협업 로봇 시스템으로 통합합니다. 
 [📄관련 문서 (🏠︎ > Digital Twin > Isaac Cortex)](https://docs.isaacsim.omniverse.nvidia.com/latest/cortex_tutorials/tutorial_cortex_1_overview.html#isaac-cortex-overview)


[![스크린샷 2025-02-19 144331](https://github.com/user-attachments/assets/4e20f13e-85c1-4efe-a974-9f278c75c8d3)](https://www.nvidia.com/en-us/on-demand/session/gtcspring22-s42693/)

- **Standalone Example**
  - [x] [🐍📝 example_command_api_main.py](https://github.com/airobotics01/redEddie/blob/main/FR3Cortex/example_command_api_main.py
)  
  - [x] [🐍📝 follow_example_main.py](https://github.com/airobotics01/redEddie/blob/main/FR3Cortex/follow_example_main.py
)  


- **Tutorial Sequence**
  - [ ] [Decider Network](https://docs.isaacsim.omniverse.nvidia.com/latest/cortex_tutorials/tutorial_cortex_4_franka_block_stacking.html#)
  - [ ] Behavior Examples: Peck Games
  - [ ] Walkthrough: Franka Block Stacking
  - [ ] ~~Walkthrough: UR10 Bin Stacking~~
  - [ ] Building Cortex Based Extensions

## 기타

- [ ] **Isaac Manipulator**  
  - FoundationPose와 DOPE를 이용해 UR10 로봇 팔로 토마토캔을 잡는 작업을 시도했으나, 실행이 원활하지 않음.  

- [ ] 익스텐션을 이용한 GUI
- [ ] Task를 세부적으로 나누기
- [ ] 컵 transition과 orientation을 제공
- [ ] 그리퍼에 토크센서 넣을 수 있는지
- [ ] 컨베이어 벨트에 스폰한 물체 집는거
