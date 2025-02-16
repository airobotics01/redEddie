from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.physx.scripts import utils
from pxr import Usd, UsdPhysics, UsdGeom

# Stage 가져오기
stage = Usd.Stage.Open(omni.usd.get_context().get_stage_url())

# 경로 설정
can_prim_path = "/World/soup_can"
usd_file = r"C:\redEddie-main\FR3\urdf\soup_can\soup_can.usd"

# USD 파일 참조 추가
add_reference_to_stage(usd_path=usd_file, prim_path=can_prim_path)

# Prim 객체 가져오기
can_prim = stage.GetPrimAtPath(can_prim_path)

# RigidBody 설정
utils.setRigidBody(can_prim, approximationShape="convexDecomposition", kinematic=False)

# 질량 설정
mass = 1.0  # 원하는 질량 값
massAPI = UsdPhysics.MassAPI.Apply(can_prim)
massAPI.GetMassAttr().Set(mass)


# TODO 크기 및 위치 조정
xformable = UsdGeom.Xformable(can_prim)
print(xformable.GetOrderedXformOps())

# existing_op = xformable.GetOrderedXformOps()[0]  # 첫 번째 xformOp 가져오기
# existing_op.Set(new_value)  # 새로운 값 설정


# xform = UsdGeom.Xformable(can_prim)
# xform.AddScaleOp().Set((10, 10, 10))  # 크기를 10배로 증가
# xform.AddTranslateOp().Set((0, 0, 10))  # Z축으로 10 단위 이동
