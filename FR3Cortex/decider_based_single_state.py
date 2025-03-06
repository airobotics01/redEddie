from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.cortex.framework.cortex_world import CortexWorld
from isaacsim.cortex.framework.df import DfNetwork, DfDecider
from isaacsim.cortex.framework.dfb import DfRobotApiContext
from robot import add_fr3_to_stage


# 1. Define a simple context
class SimpleContext(DfRobotApiContext):
    def __init__(self, robot):
        super().__init__(robot)
        self.reset()
        self.add_monitors([SimpleContext.monitor_state])

    def reset(self):
        self.iteration = 0

    def monitor_state(self):
        # Simply increments a counter to show activity
        self.iteration += 1
        print(f"Monitor state called: iteration {self.iteration}")


# 2. Define a single decider node with no children
class SingleStateDecider(DfDecider):
    def __init__(self):
        super().__init__()

    def enter(self):
        print("Entered SingleStateDecider")

    def decide(self):
        # No children to choose from, so return None
        print(f"Current iteration: {self.context.iteration}")
        return None

    def exit(self):
        print("Exited SingleStateDecider")


# 3. Create the simulation world
world = CortexWorld()
robot = world.add_robot(add_fr3_to_stage(name="fr3", prim_path="/World/fr3"))
world.scene.add_default_ground_plane()

# 4. Create and add the decider network
root_decider = SingleStateDecider()
context = SimpleContext(robot)
decider_network = DfNetwork(root_decider, context=context)
world.add_decider_network(decider_network)

# 5. Run the simulation
world.reset()
print("Starting simulation...")
world.run(simulation_app)
print("Simulation complete.")
simulation_app.close()
