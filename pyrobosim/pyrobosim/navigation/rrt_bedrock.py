import time

from pyrobosim.navigation.planner_base import PathPlannerBase
from pyrobosim.utils.motion import Path
from pyrobosim.utils.pose import Pose
from pyrobosim.utils.search_graph import SearchGraph, Node
from pyrobosim.core.world import World


class RRTPlannerBedrockImpl:

    def __init__(
        self,
        world,
        collision_distance: float,
        num_iterations: int,
        max_connection_dist: float,
    ):
        self._num_iterations = num_iterations
        self._world = world
        self._collision_distance = collision_distance
        self._max_connection_dist = max_connection_dist
        self._reset()

    def _reset(self):
        self.graph = SearchGraph(color=[0, 0, 1], color_alpha=0.5)
        self.latest_path = Path()

    def plan(self, start: Pose, goal: Pose) -> Path:
        goal_reached = False
        self._reset()
        start_node = Node(start)
        goal_node = Node(goal)
        self.graph.add_node(start_node)

        #  goal/start pose on obstacle
        if self._world.check_occupancy(goal) or self._world.check_occupancy(start):
            return self.latest_path

        if self._world.is_connectable(
            start, goal, step_dist=0.01, max_dist=self._max_connection_dist
        ):
            self.graph.add_node(goal_node)
            self.graph.add_edge(start_node, goal_node)
            goal_node.parent = start_node
            goal_reached = True

        # Qgoal //region that identifies success
        # Counter = 0 //keeps track of iterations
        # lim = n //number of iterations algorithm should run for
        # G(V,E) //Graph containing edges and vertices, initialized as empty
        # While counter < lim:
        #     Xnew  = RandomPosition()
        #     if IsInObstacle(Xnew) == True:
        #         continue
        #     Xnearest = Nearest(G(V,E),Xnew) //find nearest vertex
        #     Link = Chain(Xnew,Xnearest)
        #     G.append(Link)
        #     if Xnew in Qgoal:
        #         Return G
        # Return G
        for _ in range(self._num_iterations):
            if goal_reached:
                break
            # sample position to grow tree
            new_position = self._world.sample_free_robot_pose_uniform()
            if self._world.check_occupancy(new_position):
                continue
            # find nearest vertex from existing graph
            nearest_vertex = self.graph.nearest(new_position)
            new_node = None
            if self._world.is_connectable(
                new_position,
                nearest_vertex.pose,
                step_dist=0.01,
                max_dist=self._max_connection_dist,
            ):
                new_node = Node(new_position)
                self.graph.add_node(new_node)
                self.graph.add_edge(new_node, nearest_vertex)
                new_node.parent = nearest_vertex
            # if not connectable then extend graph in direction of sampled position
            else:
                extended_pose = self._extend(nearest_vertex.pose, new_position)
                if not self._world.check_occupancy(
                    extended_pose
                ) and self._world.is_connectable(
                    nearest_vertex.pose,
                    extended_pose,
                    step_dist=0.01,
                    max_dist=self._max_connection_dist,
                ):
                    new_node = Node(extended_pose)
                    self.graph.add_node(new_node)
                    self.graph.add_edge(new_node, nearest_vertex)
                    new_node.parent = nearest_vertex
                    new_position = extended_pose
            # check if new node is connectable to goal
            if new_node is not None:
                if self._world.is_connectable(
                    new_position,
                    goal,
                    step_dist=0.01,
                    max_dist=self._max_connection_dist,
                ):
                    goal_reached = True
                    self.graph.add_node(goal_node)
                    self.graph.add_edge(new_node, goal_node)
                    goal_node.parent = new_node

        path_to_return = self._construct_path(goal_node, start_node)
        self.latest_path = path_to_return
        return self.latest_path

    # Extends the tree in the direction of the sampled node which isn't within range
    # of the tree
    def _extend(self, start: Pose, goal: Pose) -> Pose:
        dx = goal.x - start.x
        dy = goal.y - start.y
        distance = (dx**2 + dy**2) ** 0.5

        # Avoid division by zero
        if distance == 0:
            return Pose(x=start.x, y=start.y)

        # Normalize the direction vector
        dx_normalized = dx / distance
        dy_normalized = dy / distance

        # Calculate new pose within max_connection_dist
        new_x = start.x + dx_normalized * self._max_connection_dist
        new_y = start.y + dy_normalized * self._max_connection_dist

        return Pose(x=new_x, y=new_y)

    # simply traverses the graph from goal node via the parent link
    def _construct_path(self, goal_node, start_node) -> Path:
        if goal_node is None or start_node is None:
            return Path()
        if goal_node.pose == start_node.pose:
            return Path(poses=[start_node.pose])
        path_to_return = []
        trav_node = goal_node
        while trav_node and trav_node != start_node:
            path_to_return.append(trav_node.pose)
            trav_node = trav_node.parent
        if trav_node == start_node:
            path_to_return.append(start_node.pose)
        path_to_return.reverse()
        return Path(poses=path_to_return)

    def get_graphs(self):
        return [self.graph]


class RRTBedrockPlanner(PathPlannerBase):
    def __init__(self, **planner_config):
        super().__init__()
        self.impl = RRTPlannerBedrockImpl(**planner_config)

    def plan(self, start, goal):
        start_time = time.time()
        self.latest_path = self.impl.plan(start, goal)
        self.planning_time = time.time() - start_time
        self.graphs = self.impl.get_graphs()
        return self.latest_path
