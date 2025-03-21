from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})


from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api import World
from omni.isaac.nucleus import get_assets_root_path


import numpy as np
from franka import FR3
from ros2_actiongraph import subscribe_joint_command_with_action_graph
from tasks.pick_place import PickPlace
from controllers.pick_place_controller import PickPlaceController

my_world = World()

assets_root_path = get_assets_root_path()

add_reference_to_stage(
    usd_path=f"{assets_root_path}/Isaac/Environments/Simple_Room/simple_room.usd",
    prim_path="/World/SimpleRoom",
)
add_reference_to_stage(
    usd_path=f"{assets_root_path}/Isaac/Robots/Franka/FR3/fr3.usd",
    prim_path="/World/FR3",
)

my_world.reset()

import omni.graph.core as og
import isaacsim.ros2.bridge

og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            (
                "ArticulationController",
                "isaacsim.core.nodes.IsaacArticulationController",
            ),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
            (
                "ReadSimTime.outputs:simulationTime",
                "PublishJointState.inputs:timeStamp",
            ),
            (
                "SubscribeJointState.outputs:jointNames",
                "ArticulationController.inputs:jointNames",
            ),
            (
                "SubscribeJointState.outputs:positionCommand",
                "ArticulationController.inputs:positionCommand",
            ),
            (
                "SubscribeJointState.outputs:velocityCommand",
                "ArticulationController.inputs:velocityCommand",
            ),
            (
                "SubscribeJointState.outputs:effortCommand",
                "ArticulationController.inputs:effortCommand",
            ),
            ("ROS2Context.outputs:context", "PublishJointState.inputs:context"),
            ("ROS2Context.outputs:context", "SubscribeJointState.inputs:context"),
        ],
        og.Controller.Keys.SET_VALUES: [
            # Providing path to /World/FR3 robot to Articulation Controller node
            # Providing the robot path is equivalent to setting the targetPrim in Articulation Controller node
            # ("ArticulationController.inputs:usePath", True),      # if you are using an older version of Isaac Sim, you may need to uncomment this line
            ("ArticulationController.inputs:robotPath", "/World/FR3"),
            ("PublishJointState.inputs:targetPrim", "/World/FR3"),
            ("ROS2Context.inputs:domain_id", 101),
            ("ROS2Context.inputs:useDomainIDEnvVar", False),
        ],
    },
)

my_world.reset()

simulation_app.update()

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
