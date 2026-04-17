# check.py
import random
import networkx as nx
import logging
import time
from copy import deepcopy

from ex2 import ids, OptimalRobotAgent, RobotAgent, InfiniteRobotAgent, ExampleRobotAgent
from inputs import finite_inputs, optimal_inputs, pi_vi_inputs

RESET_PENALTY = 2
PICK_UP_REWARD = 2
EMERGENCY_CHARGE_PENALTY = 5

INIT_TIME_LIMIT = 300
TURN_TIME_LIMIT = 5

INFINITE_TEST_STEPS = 40  # how many steps to simulate for infinite instances

K_MAP = "map"
K_ROBOT = "robot"
K_ROBOVACS = "robovacs"
K_PICKUPS = "pick_up_locations"
K_CHARGERS = "charging_stations"
K_UNEVEN = "uneven_floor"
K_TURNS = "turns_to_go"

K_ROB_LOC = "location"
K_ROB_BAT = "battery"


def initiate_agent(state):
    """
    Robot-style: choose agent class based on flags in input.
    """
    if state.get("optimal", False):
        if state.get("infinite", False):
            return InfiniteRobotAgent(state, state["gamma"])
        return OptimalRobotAgent(state)
    return RobotAgent(state)


class EndOfGame(Exception):
    pass


class RobotStochasticProblem:
    def __init__(self, an_input):
        self.initial_state = deepcopy(an_input)
        self.state = deepcopy(an_input)

        self._ensure_runtime_fields(self.initial_state)
        self._ensure_runtime_fields(self.state)

        self.graph = self.build_graph()

        start = time.perf_counter()
        self.agent = initiate_agent(deepcopy(self.state))

        end = time.perf_counter()
        if end - start > INIT_TIME_LIMIT:
            logging.critical("timed out on constructor")
            raise TimeoutError

        self.score = 0

    # -----------------------------
    # Runtime fields
    # -----------------------------
    @staticmethod
    def _ensure_runtime_fields(state):
        robot = state[K_ROBOT]
        if "location" not in robot:
            robot["location"] = robot["start"]
        if "battery" not in robot:
            robot["battery"] = robot["starting_moves_left"]

        for charging_station in state.get(K_CHARGERS, []):
            if "cooldown" not in charging_station:
                charging_station["cooldown"] = 0

        for robovac in state.get(K_ROBOVACS, []):
            if "index" not in robovac:
                robovac["index"] = 0
            if "direction" not in robovac:
                robovac["direction"] = "F"
    # -----------------------------
    # Main loop
    # -----------------------------
    def run_round(self):
        if self.state.get("infinite", False):
            steps = INFINITE_TEST_STEPS
            while steps > 0:
                self._step_once()
                steps -= 1
            self.terminate_execution()
        else:
            while self.state[K_TURNS] > 0:
                self._step_once()
            self.terminate_execution()

    def _step_once(self):
        start = time.perf_counter()
        action = self.agent.act(deepcopy(self.state))
        end = time.perf_counter()
        if end - start > TURN_TIME_LIMIT:
            logging.critical("timed out on an action")
            raise TimeoutError

        if not self.is_action_legal(action):
            logging.critical("Illegal action: %r", action)
            raise RuntimeError(f"Illegal action: {action}")

        self.result(action)

    # -----------------------------
    # Helpers
    # -----------------------------
    def _in_bounds(self, loc):
        row, col = loc
        grid = self.state[K_MAP]
        return 0 <= row < len(grid) and 0 <= col < len(grid[0])

    def _passable(self, loc):
        row, col = loc
        return self.state[K_MAP][row][col] != "I"

    @staticmethod
    def _manhattan(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def _move_cost(self, dest):
        rob = self.state[K_ROBOT]
        uneven_tiles = set(self.state.get(K_UNEVEN, []))
        if dest in uneven_tiles:
            return int(rob.get("uneven_floor_penalty", 1))
        if rob[K_ROB_LOC] in uneven_tiles:
            return int(rob.get("uneven_floor_penalty", 1))
        return 1

    def _charger_at(self, loc):
        for cs in self.state.get(K_CHARGERS, []):
            if cs.get("location") == loc:
                return cs
        return None

    def _pickup_by_name(self, name):
        for p in self.state.get(K_PICKUPS, []):
            if p.get("name") == name:
                return p
        return None

    def _robovac_loc(self, rv):
        path = rv["path"]
        idx = int(rv["index"])
        idx = max(0, min(idx, len(path) - 1))
        return path[idx]

    # -----------------------------
    # Legality
    # -----------------------------
    def is_action_legal(self, action):
        robot = self.state[K_ROBOT]
        robot_loc = robot[K_ROB_LOC]
        battery = robot[K_ROB_BAT]
        infinite = self.state.get("infinite", False)

        # reset/terminate: illegal in infinite mode
        if action in (("reset",), ("terminate",)):
            return not infinite

        if not isinstance(action, tuple) or len(action) == 0:
            return False

        # battery depleted: only emergency_charge (or reset / terminate) allowed
        if battery < 0:
            return action == ("emergency_charge",)

        if action == ("wait",):
            return True

        if action[0] == "move":
            if len(action) != 2 or not isinstance(action[1], tuple) or len(action[1]) != 2:
                return False
            dest = action[1]
            if not self._in_bounds(dest) or not self._passable(dest):
                return False
            if self._manhattan(robot_loc, dest) != 1:
                return False
            if battery == 0:
                return False
            return True  # battery >= self._move_cost(dest)

        if action == ("charge",):
            cs = self._charger_at(robot_loc)
            if cs is None:
                return False
            # cooldown ignored in infinite mode
            if infinite:
                return True
            return int(cs.get("cooldown", 0)) == 0

        if action == ("emergency_charge",):
            return battery <= 0

        if action[0] == "pick-up":
            if len(action) != 3:
                return False
            name, loc = action[1], action[2]
            if not isinstance(name, str) or not isinstance(loc, tuple) or len(loc) != 2:
                return False
            if loc != robot_loc:
                return False
            p = self._pickup_by_name(name)
            if p is None:
                return False
            return p.get("location") == loc == robot_loc

        return False

    # -----------------------------
    # Transition
    # -----------------------------
    def result(self, action):
        self.apply(action)
        if action != ("reset",):
            self.environment_step()
        self.check_collision_with_robovacs()

    def apply(self, action):
        if action == ("reset",):
            self.reset_environment()
            return
        if action == ("terminate",):
            self.terminate_execution()
        self.apply_atomic_action(action)

    def apply_atomic_action(self, action):
        robot = self.state[K_ROBOT]
        infinite = self.state.get("infinite", False)

        if action[0] == "move":
            dest = action[1]
            robot["location"] = dest
            robot["battery"] -= self._move_cost(dest)
            return

        if action == ("wait",):
            return

        if action == ("charge",):
            cs = self._charger_at(robot["location"])
            robot["battery"] = min(
                robot["battery"] + int(cs["charge_amount"]),
                int(robot["maximum_moves_left_possible"]),
            )
            # cooldown ignored in infinite mode
            if not infinite:
                cs["cooldown"] = int(cs["charge_wait"])
            return

        if action == ("emergency_charge",):
            self.score -= EMERGENCY_CHARGE_PENALTY
            robot["battery"] = int(robot["maximum_moves_left_possible"])
            return

        if action[0] == "pick-up":
            self.score += PICK_UP_REWARD
            return

        raise NotImplementedError(action)

    def environment_step(self):
        # pickups move stochastically
        for p in self.state.get(K_PICKUPS, []):
            if random.random() < float(p["move_probability"]):
                p["location"] = random.choice(p["possible_locations"])

        # robovacs stochastic step
        for rv in self.state.get(K_ROBOVACS, []):
            path = rv["path"]
            if len(path) <= 1:
                continue
            idx = int(rv["index"])
            direction = rv.get("direction", "F")

            if idx == 0:
                rv["index"] = 0 if random.random() < 0.25 else 1
            elif idx == len(path) - 1:
                rv["index"] = idx if random.random() < 0.25 else idx - 1
            else:
                u = random.random()
                if direction == "F":
                    rv["index"] = idx - 1 if u < 0.25 else (idx if u < 0.50 else idx + 1)
                else:
                    rv["index"] = idx + 1 if u < 0.25 else (idx if u < 0.50 else idx - 1)

            if rv["index"] == 0:
                rv["direction"] = "F"
            elif rv["index"] == len(path) - 1:
                rv["direction"] = "B"

        # cooldown ticks (ignored in infinite mode)
        if not self.state.get("infinite", False):
            for cs in self.state.get(K_CHARGERS, []):
                cs["cooldown"] = max(0, int(cs.get("cooldown", 0)) - 1)

        # turns only in finite mode
        if K_TURNS in self.state:
            self.state[K_TURNS] -= 1

    def check_collision_with_robovacs(self):
        robot = self.state[K_ROBOT]
        dmg = int(robot.get("robovac_battery_damage", 0))
        if dmg <= 0:
            return
        robot_loc = robot["location"]
        hits = 0
        for rv in self.state.get(K_ROBOVACS, []):
            if self._robovac_loc(rv) == robot_loc:
                hits += 1
        if hits:
            robot["battery"] -= hits * dmg

    def reset_environment(self):
        self.state[K_ROBOT] = deepcopy(self.initial_state[K_ROBOT])
        self.state[K_ROBOVACS] = deepcopy(self.initial_state.get(K_ROBOVACS, []))
        self.state[K_PICKUPS] = deepcopy(self.initial_state.get(K_PICKUPS, []))
        self.state[K_CHARGERS] = deepcopy(self.initial_state.get(K_CHARGERS, []))
        self.state[K_UNEVEN] = deepcopy(self.initial_state.get(K_UNEVEN, []))
        self._ensure_runtime_fields(self.state)
        self.state[K_TURNS] -= 1
        self.score -= RESET_PENALTY

    def terminate_execution(self):
        print(f"End of game, your score is {self.score}!")
        print("-----------------------------------")
        raise EndOfGame

    def build_graph(self):
        grid = self.initial_state[K_MAP]
        n_rows, n_cols = len(grid), len(grid[0])
        g = nx.grid_graph((n_cols, n_rows))
        nodes_to_remove = []
        for node in g:
            if grid[node[0]][node[1]] == "I":
                nodes_to_remove.append(node)
        for node in nodes_to_remove:
            g.remove_node(node)
        return g


def run_inputs(input_list):
    for an_input in input_list:
        try:
            p = RobotStochasticProblem(an_input)
            p.run_round()
        except EndOfGame:
            continue


def main():
    print(f"IDS: {ids}")

    print("\n--- RobotAgent suite (finite_inputs) ---")
    run_inputs(finite_inputs)

    print("\n--- OptimalRobotAgent suite (optimal_inputs) ---")
    run_inputs(optimal_inputs)

    print("\n--- InfiniteRobotAgent suite (pi_vi_inputs) ---")
    run_inputs(pi_vi_inputs)


if __name__ == "__main__":
    main()
