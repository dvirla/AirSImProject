import numpy as np
from collections import defaultdict
from itertools import permutations
from utils import hashabledict
from copy import copy
from GraphVisualize import GraphVisualization

np.random.seed(0)


class MonteCarloTreeSearchNode():
    graph = GraphVisualization()
    id = -1
    num_packages = 20
    packages_locations = np.random.randint(-100, 100, size=(num_packages, 2))

    def __init__(self, state, goal, parent=None, parent_action=None):
        self.state = state
        self.parent = parent
        self.parent_action = parent_action
        self.children = []
        self._number_of_visits = 0
        MonteCarloTreeSearchNode.id += 1
        self.id = MonteCarloTreeSearchNode.id
        if parent is None:
            self.depth = 0
            MonteCarloTreeSearchNode.graph.addEdge(0, self.id)
        else:
            self.depth = self.parent.depth + 1
            MonteCarloTreeSearchNode.graph.addEdge(self.parent.id, self.id)
        self.minimal_distance = float('inf')
        self._untried_actions = self.untried_actions()
        self.goal = goal

        return

    def untried_actions(self):
        self._untried_actions = self.get_legal_actions(self.state)
        return self._untried_actions

    def get_total_distances(self):
        total = 0
        current = self
        while current.parent is not None:
            parent = current.parent
            for drone in self.state['drones']:
                pos = np.array([0, 0])
                parent_pos = np.array([0, 0])
                if current.state['drones'][drone] != 0:
                    pos = MonteCarloTreeSearchNode.packages_locations[current.state['drones'][drone] - 1]
                if parent.state['drones'][drone] != 0:
                    parent_pos = MonteCarloTreeSearchNode.packages_locations[parent.state['drones'][drone] - 1]
                total += np.sqrt(np.sum(np.square(pos - parent_pos)))
            current = parent
        # for drone, destinations in self.state['drones'].items():
        #     drone_loc = np.array([0, 0])
        #     if self.parent.state['drones'][drone] != 0:
        #         drone_loc = MonteCarloTreeSearchNode.packages_locations[self.parent.state['drones'][drone] - 1]
        #     if type(destinations) == int:
        #         destinations = [destinations]
        #     for dest in destinations:
        #         if dest > 0:
        #             dest_loc = MonteCarloTreeSearchNode.packages_locations[dest-1]
        #             total += np.linalg.norm(dest_loc - drone_loc)
        #             drone_loc = dest_loc
        return total

    @staticmethod
    def get_legal_actions(state):
        """
        :param state: drones locations & package locations: -1 for destination, positive integers are drones, 0 destination
        1. for each drone -> if at home & have room for package: check packages -> if reachable: lift go to dest, else: stay.
        2. for each drone -> if at destination & have package: go to next destination, else: go home.
        """
        drone_package_pairs = defaultdict(list)
        free_packages = []
        available_drones = []
        unavailable_drones = []
        home = []
        for package, drone in state['packages'].items():
            if drone > 0:
                drone_package_pairs[drone].append(package)
            elif drone == 0:
                free_packages.append(package)

        for drone, location in state['drones'].items():
            if location == 0 and len(drone_package_pairs[drone]) < min(2, len(free_packages)):
                available_drones.append(drone)
            elif location != 0 and len(drone_package_pairs[drone]) == 0:
                home.append(drone)
            else:
                unavailable_drones.append(drone)
        actions = []
        for perm in permutations(free_packages, min(len(available_drones), len(free_packages))):
            action = hashabledict({drone: 0 for drone in state['drones']})
            for i, drone in enumerate(available_drones):
                if i >= len(free_packages):
                    break
                action[drone] = -perm[i]  # -x    Pickup package x
            for drone in unavailable_drones:
                if drone_package_pairs[drone]:
                    action[drone] = drone_package_pairs[drone][0]  # x     Go to location of package x
            actions.append(action)
        actions = list(set(actions))
        # print(len(actions))
        return actions

    def q(self):
        # return -self.depth
        # t = self.get_total_distances()
        # print(t)
        # return -t
        return -self.minimal_distance
        # return np.random.randint(1, 1000)

    def n(self):
        return self._number_of_visits

    def expand(self):
        action = self._untried_actions.pop()
        next_state = self.move(self.state, action)
        child_node = MonteCarloTreeSearchNode(
            next_state, self.goal, parent=self, parent_action=action)

        self.children.append(child_node)
        return child_node

    @staticmethod
    def move(state, action):
        new_state = hashabledict()
        new_state['drones'] = hashabledict({drone: location for drone, location in state['drones'].items()})
        new_state['packages'] = hashabledict({package: location for package, location in state['packages'].items()})
        for drone, operation in action.items():
            package = abs(operation)
            if operation == 0:
                new_state['drones'][drone] = 0
            elif operation > 0:
                new_state['drones'][drone] = operation
                new_state['packages'][package] = -1
            else:
                new_state['packages'][package] = drone
        return new_state

    def is_terminal_node(self):
        return self.state == self.goal

    def rollout(self):
        current_rollout_state = copy(self.state)
        distance = self.get_total_distances()
        while current_rollout_state != self.goal:
            possible_moves = self.get_legal_actions(current_rollout_state)
            action = self.rollout_policy(possible_moves)
            new_rollout_state = self.move(current_rollout_state, action)
            for drone in self.state['drones']:
                pos = np.array([0, 0])
                parent_pos = np.array([0, 0])
                if new_rollout_state['drones'][drone] != 0:
                    pos = MonteCarloTreeSearchNode.packages_locations[new_rollout_state['drones'][drone] - 1]
                if current_rollout_state['drones'][drone] != 0:
                    parent_pos = MonteCarloTreeSearchNode.packages_locations[current_rollout_state['drones'][drone] - 1]
                distance += np.sqrt(np.sum(np.square(pos - parent_pos)))
            current_rollout_state = new_rollout_state
        return distance

    def backpropagate(self, result):
        self._number_of_visits += 1.
        self.minimal_distance = min(self.minimal_distance, result)
        if self.parent:
            self.parent.backpropagate(result)

    def is_fully_expanded(self):
        return len(self._untried_actions) == 0

    def best_child(self, c_param=0.1):
        choices_weights = [(c.q() / c.n()) + c_param * np.sqrt((2 * np.log(self.n()) / c.n())) for c in self.children]
        return self.children[np.argmax(choices_weights)]

    @staticmethod
    def rollout_policy(possible_moves):
        return possible_moves[np.random.randint(len(possible_moves))]

    def _tree_policy(self):
        current_node = self
        while not current_node.is_terminal_node():

            if not current_node.is_fully_expanded():
                return current_node.expand()
            else:
                current_node = current_node.best_child()
        return current_node

    def best_action(self):
        simulation_no = 100

        for i in range(simulation_no):
            v = self._tree_policy()
            reward = v.rollout()
            v.backpropagate(reward)

        return self.best_child(c_param=0.)