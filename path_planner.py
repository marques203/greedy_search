from grid import Node, NodeGrid
from math import inf
import heapq


class PathPlanner(object):
    """
    Represents a path planner, which may use Dijkstra, Greedy Search or A* to plan a path.
    """
    def __init__(self, cost_map):
        """
        Creates a new path planner for a given cost map.

        :param cost_map: cost used in this path planner.
        :type cost_map: CostMap.
        """
        self.cost_map = cost_map
        self.node_grid = NodeGrid(cost_map)

    @staticmethod
    def construct_path(goal_node):
        """
        Extracts the path after a planning was executed.

        :param goal_node: node of the grid where the goal was found.
        :type goal_node: Node.
        :return: the path as a sequence of (x, y) positions: [(x1,y1),(x2,y2),(x3,y3),...,(xn,yn)].
        :rtype: list of tuples.
        """
        node = goal_node
        # Since we are going from the goal node to the start node following the parents, we
        # are transversing the path in reverse
        reversed_path = []
        while node is not None:
            reversed_path.append(node.get_position())
            node = node.parent
        return reversed_path[::-1]  # This syntax creates the reverse list

    def dijkstra(self, start_position, goal_position):
        """
        Plans a path using the Dijkstra algorithm.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
		# Todo: implement the Dijkstra algorithm
        self.node_grid.reset()
        pq = []
        start = self.node_grid.get_node(start_position[0], start_position[1])
        start.g = 0
        heapq.heappush(pq, (start.g, start))
        goal_node=False
        while not len(pq) == 0 and goal_node==False:
           aux = heapq.heappop(pq)
           node=aux[1]
           node.g = aux[0]
           node.closed=True
           if node.get_position() == goal_position:
               goal_node=node
           for successor in self.node_grid.get_successors(node.i, node.j):
             suc = self.node_grid.get_node(successor[0], successor[1])
             if suc.g > node.g + self.cost_map.get_edge_cost(node.get_position(), suc.get_position()) and suc.closed==False:
                suc.g = node.g + self.cost_map.get_edge_cost(node.get_position(), suc.get_position())
                suc.parent = node
                heapq.heappush(pq, (suc.g, suc))
		# The first return is the path as sequence of tuples (as returned by the method construct_path())
		# The second return is the cost of the path

        return PathPlanner.construct_path(goal_node), goal_node.g

    def greedy(self, start_position, goal_position):
        """
        Plans a path using greedy search.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
		# Todo: implement the Greedy Search algorithm
		# The first return is the path as sequence of tuples (as returned by the method construct_path())
		# The second return is the cost of the path
        self.node_grid.reset()
        pq = []
        start = self.node_grid.get_node(start_position[0], start_position[1])
        start.g = 0
        start.f = start.distance_to(goal_position[0], goal_position[1])
        heapq.heappush(pq, (start.f, start))
        while not len(pq)==0:
            aux = heapq.heappop(pq)
            node = aux[1]
            node.f = aux[0]
            node.closed=True
            for s in self.node_grid.get_successors(node.i, node.j):
                successor = self.node_grid.get_node(s[0], s[1])
                if not successor.closed:
                    successor.parent = node
                    successor.g = node.g + self.cost_map.get_edge_cost(node.get_position(), successor.get_position())
                    if successor.get_position() == goal_position:
                        return PathPlanner.construct_path(successor), successor.g
                    successor.f = successor.distance_to(goal_position[0], goal_position[1])
                    successor.closed = True
                    heapq.heappush(pq, (successor.f, successor))

    def a_star(self, start_position, goal_position):
        """
        Plans a path using A*.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
		# Todo: implement the A* algorithm
		# The first return is the path as sequence of tuples (as returned by the method construct_path())
		# The second return is the cost of the path
        self.node_grid.reset()
        # Initialize node.g and node.f to inf for all nodes
        pq = []
        start = self.node_grid.get_node(start_position[0], start_position[1])
        start.g = 0
        start.f = start.distance_to(goal_position[0], goal_position[1])
        heapq.heappush(pq, (start.f, start))
        while not len(pq)==0:
          aux = heapq.heappop(pq)
          node = aux[1]
          node.f = aux[0]
          node.closed = True
          if node.get_position() == goal_position:
              return PathPlanner.construct_path(node), node.g
          for s in self.node_grid.get_successors(node.i, node.j):
            successor = self.node_grid.get_node(s[0], s[1])
            if successor.f > node.g + self.cost_map.get_edge_cost(node.get_position(), successor.get_position()) + successor.distance_to(goal_position[0], goal_position[1]) and not successor.closed:
                successor.g = node.g + self.cost_map.get_edge_cost(node.get_position(), successor.get_position())
                successor.f = successor.g + successor.distance_to(goal_position[0], goal_position[1])
                successor.parent = node
                heapq.heappush(pq, (successor.f, successor))
