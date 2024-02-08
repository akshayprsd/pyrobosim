#!/usr/bin/env python3
import os

from pyrobosim.core import WorldYamlLoader
from pyrobosim.gui import start_gui
from pyrobosim.navigation import PathPlanner
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose

world_file = os.path.join(get_data_folder(), "test_world.yaml")
world = WorldYamlLoader().from_yaml(world_file)


def test_rrt_bedrock():
    planner_config = {
        "world": world,
        "collision_distance": 0.025,
        "num_iterations": 1000,
        "max_connection_dist": 0.5,
    }
    rrt_bedrock = PathPlanner("rrt_bedrock", **planner_config)
    start = Pose(x=-0.5, y=-0.5)
    goal = Pose(x=3.0, y=3.0)

    robot = world.robots[0]
    robot.set_pose(start)
    robot.set_path_planner(rrt_bedrock)
    result = robot.plan_path(start, goal)
    rrt_bedrock.info()

if __name__ == "__main__":
    test_rrt_bedrock()
    start_gui(world)
