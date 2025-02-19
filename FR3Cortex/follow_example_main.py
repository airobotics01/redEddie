from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api.objects import VisualSphere
from isaacsim.cortex.framework.cortex_world import CortexWorld
from isaacsim.cortex.framework.df import DfNetwork, DfState, DfStateMachineDecider
from isaacsim.cortex.framework.dfb import DfBasicContext
from robot import add_fr3_to_stage


class FollowState(DfState):
    """The context object is available as self.context. We have access to everything in the context
    object, which in this case is everything in the robot object (the command API and the follow
    sphere).
    """

    @property
    def robot(self):
        return self.context.robot

    @property
    def follow_sphere(self):
        return self.context.robot.follow_sphere

    def enter(self):
        self.robot.gripper.close()
        self.follow_sphere.set_world_pose(*self.robot.arm.get_fk_pq().as_tuple())

    def step(self):
        target_position, _ = self.follow_sphere.get_world_pose()
        self.robot.arm.send_end_effector(target_position=target_position)
        return self  # Always transition back to this state.


def main():
    world = CortexWorld()
    robot = world.add_robot(add_fr3_to_stage(name="my_fr3", prim_path="/World/fr3"))

    # Add a sphere to the scene to follow, and store it off in a new member as part of the robot.
    robot.follow_sphere = world.scene.add(
        VisualSphere(
            name="follow_sphere",
            prim_path="/World/FollowSphere",
            radius=0.02,
            color=np.array([0.7, 0.0, 0.7]),
        )
    )
    world.scene.add_default_ground_plane()

    # Add a simple state machine decider network with the single state defined above. This state
    # will be persistently stepped because it always returns itself.
    world.add_decider_network(
        DfNetwork(DfStateMachineDecider(FollowState()), context=DfBasicContext(robot))
    )

    world.run(simulation_app)
    simulation_app.close()


if __name__ == "__main__":
    main()
