from __future__ import print_function

import yaml
from node import Node
from utils import Utils
from geometry_msgs.msg import Point

class TopologicalPlanner(object):

    """Planner for topological nodes"""

    def __init__(self, network_file):
        with open(network_file, 'r') as file_obj:
            network = yaml.safe_load(file_obj)
        self.nodes = {node['id']:Node.from_dict(node) for node in network['nodes']}
        self._initialise_neighbours(network['connections'])

    def _initialise_neighbours(self, connections):
        self.neighbours = {}
        for node_id in self.nodes:
            neighbour_list = []
            for connection in connections:
                if node_id in connection:
                    neighbour_list.append(connection[0] if node_id == connection[1] else connection[1])
            self.neighbours[node_id] = neighbour_list

    def plan(self, start=(0.0, 0.0), goal=(0.0, 0.0)):
        """Plan a path from start node to goal

        :start: tuple of 2 float
        :goal: tuple of 2 float
        :returns: list of Point

        """
        start_node = self.get_nearest_topological_point(*start)
        goal_node = self.get_nearest_topological_point(*goal)
        return self.plan_path(start_node, goal_node)

    def plan_path(self, start_node, goal_node, search_type='bfs'):
        """Plan a path from start node to goal

        :start_node: int
        :goal_node: int
        :returns: list of Point

        """
        topological_path = self.search(start_node, goal_node, search_type)
        if topological_path is None:
            return None

        node_path = [self.nodes[node_id] for node_id in topological_path]
        point_path = [Point(x=n.x, y=n.y) for n in node_path]
        return point_path

    def get_nearest_topological_point(self, x, y):
        """
        Finds nearest topological point from network to the given (x, y) point

        :x: float
        :y: float
        :returns: int

        """
        nearest_node = self.nodes[self.nodes.keys()[0]]
        min_dist = float('inf')
        for node in list(self.nodes.values()):
            dist = Utils.get_distance_between_points((node.x, node.y), (x, y))
            if dist < min_dist:
                min_dist = dist
                nearest_node = node.id
        return nearest_node

    def search(self, start, goal, search_type):
        fringe = [start]
        visited = []
        parent = {}
        while len(fringe) > 0:
            if search_type == 'bfs':
                curr_node = fringe.pop(0)
            else:
                curr_node = fringe.pop()
            if curr_node == goal:
                topological_path = [goal]
                while topological_path[-1] in parent:
                    topological_path.append(parent[topological_path[-1]])
                return topological_path[::-1]

            visited.append(curr_node)
            for n in self.neighbours[curr_node]:
                if n not in visited:
                    fringe.append(n)
                    parent[n] = curr_node
