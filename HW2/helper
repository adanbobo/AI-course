from copy import deepcopy
import random
import networkx as nx
import time
import logging
import itertools
import math

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


"""
This class is intended to help you with calculating the distribution of states from every action you can take.
Here are some instructions on how to use it:

1. Initialize the class with an input (one of the inputs defined in inputs.py).

2. The class has a function that converts states to hashable tuples and vice versa. The informations in the states is the minimal necessary to
define a state (robot location, battery level, robovac indices and directions, pickup locations, charger cooldowns).

3. In order to access all possible states, you can use the attribute method all_state_tuples, which is a list of all possible state tuples.

4. The functions _tuple_to_state and _state_to_tuple can be used to convert between state dictionaries (like the input) and state tuples.
Notice that the state tuples do not include the "turns_to_go" field, so when converting from tuple to state, you need to provide the number of turns
to go as an additional argument.

5. The class has a method that can calculate all possible legal actions from a given state. In order to use this method,
call pos_actions_from_state(self, state) from a true state, or pos_actions_from_state_tuple(self, state_tuple) from a state tuple.

6. The class has a method that, given a state and an action, will calculate all next possible states and their probabilities. In order to use this method,
call get_next_states_probs(self, state, action) from a true state and action, or get_next_tuple_states_probs(self, state_tuple, action) from a
state tuple and an action.

7. You can import this class in the ex2 file. 
"""
class HelperClass:
    def __init__(self, an_input):
        self.initial_state = deepcopy(an_input)
        self.state = deepcopy(an_input)

        self._ensure_runtime_fields(self.initial_state)
        self._ensure_runtime_fields(self.state)

        self.graph = self.build_graph()
        return

    def calculate_all_state_tuples(self):
        # This function calculates all possible states
        # It is not optimized and may take a long time for large instances
        # Should not include duplicate states, i.e., states that are identical in all relevant aspects.
        # Turns to go should not be included in the state uniqueness.
        # The changing properties of states are: robot location, robot battery level, robovac indices, robovac directions, pickup locations, charger cooldowns

        robot_pos_locs = list(self.graph.nodes)
        robot_pos_bat = list(range(-1, int(self.initial_state["robot"]["maximum_moves_left_possible"]) + 1))

        robovac_pos_indices_and_directions = []
        for rv in self.initial_state.get("robovacs", []):
            path_len = len(rv["path"])
            indices_and_directions = []
            for idx in range(path_len):
                if path_len == 1:
                    indices_and_directions.append((0, "F"))
                elif idx == 0:
                    indices_and_directions.append((0, "F"))
                elif idx == path_len - 1:
                    indices_and_directions.append((idx, "B"))
                else:
                    indices_and_directions.append((idx, "F"))
                    indices_and_directions.append((idx, "B"))
            robovac_pos_indices_and_directions.append(indices_and_directions)
        
        combined_robovac_positions = list(itertools.product(*robovac_pos_indices_and_directions))

        charger_pos_cooldowns = []
        for cs in self.initial_state.get("charging_stations", []):
            max_cd = int(cs.get("charge_wait", 0))
            charger_pos_cooldowns.append(list(range(0, max_cd + 1)))
        
        combined_charger_cooldowns = list(itertools.product(*charger_pos_cooldowns))

        pickup_pos_locations = []
        for p in self.initial_state.get("pick_up_locations", []):
            possible_locs = p.get("possible_locations", [])
            pickup_pos_locations.append(possible_locs)
        
        combined_pickup_locations = list(itertools.product(*pickup_pos_locations))

        all_states_tuples_list = []

        for rob_loc in robot_pos_locs:
            for rob_bat in robot_pos_bat:
                for robovac_comb in combined_robovac_positions:
                    for charger_cd_comb in combined_charger_cooldowns:
                        for pickup_loc_comb in combined_pickup_locations:
                            state_tuple = (
                                rob_loc,
                                rob_bat,
                                tuple(robovac_comb),
                                tuple(pickup_loc_comb),
                                tuple(
                                    (self.initial_state.get("charging_stations", [])[i]["location"], charger_cd_comb[i])
                                    for i in range(len(charger_cd_comb))
                                ),
                            )
                            all_states_tuples_list.append(state_tuple)
                
        return all_states_tuples_list

    def _tuple_to_state(self, state_tuple, turns_to_go):
        # Converts a hashable tuple representation back to a state dictionary
        # The number of turns to go is passed separately
        rob_loc, rob_bat, robovac_tuples, pickup_tuples, charger_tuples = state_tuple
        state = deepcopy(self.initial_state)

        state["robot"]["location"] = rob_loc
        state["robot"]["battery"] = rob_bat

        state["robovacs"] = []
        for i in range(len(robovac_tuples)):
            index, direction = robovac_tuples[i]
            rv = self.initial_state.get("robovacs", [])[i]
            new_rv = {**rv, "index": index, "direction": direction}
            state["robovacs"].append(new_rv)
        
        state["pick_up_locations"] = []
        for i in range(len(pickup_tuples)):
            loc = pickup_tuples[i]
            p = self.initial_state.get("pick_up_locations", [])[i]
            new_p = {**p, "location": loc}
            state["pick_up_locations"].append(new_p)
        
        state["charging_stations"] = []
        for i, cs_tuple in enumerate(charger_tuples):
            location, cooldown = cs_tuple
            cs = self.initial_state.get("charging_stations", [])[i]
            new_cs = {**cs, "location": location, "cooldown": cooldown}
            state["charging_stations"].append(new_cs)
        
        if turns_to_go is not None:
            state["turns_to_go"] = turns_to_go
        
        return state

    def _state_to_tuple(self, state):
        # Converts a state dictionary to a hashable tuple representation
        # The state tuple includes only the relevant changing properties
        # Which are robot location, robot battery level, robovac indices, robovac directions, pickup locations, charger cooldowns, in this order
        rob = state[K_ROBOT]
        robovac_tuples = tuple(
            (rv["index"], rv.get("direction", "F")) for rv in state.get(K_ROBOVACS, [])
        )
        pickup_tuples = tuple(
            (p["location"]) for p in state.get(K_PICKUPS, [])
        )
        charger_tuples = tuple(
            (cs["location"], cs.get("cooldown", 0)) for cs in state.get(K_CHARGERS, [])
        )

        if rob[K_ROB_BAT] < -1:
            rob_bat = -1
        else:
            rob_bat = rob[K_ROB_BAT]

        return (
            rob[K_ROB_LOC],
            rob_bat,
            robovac_tuples,
            pickup_tuples,
            charger_tuples,
        )
    
    def pos_actions_from_state(self, state):
        # Returns a list of all possible legal actions from the given state
        self.state = state

        rob_loc = state["robot"]["location"]
        x, y = rob_loc
        pos_act_candidates = [("wait",), ("charge",), ("emergency_charge",), ("reset",), ("terminate",)]

        # Add possible move actions
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            new_loc = (x + dx, y + dy)
            pos_act_candidates.append(("move", new_loc))
        
        # Add possible pick-up actions
        for p in state.get("pick_up_locations", []):
            name = p["name"]
            loc = p["location"]
            pos_act_candidates.append(("pick-up", name, loc))
        
        pos_act = []
        
        # Check legality of each candidate action
        for act in pos_act_candidates:
            if self.is_action_legal(state, act):
                pos_act.append(act)
        
        return pos_act
    
    def pos_actions_from_state_tuple(self, state_tuple):
        # Converts state tuple to state dictionary, then calls pos_actions_from_state
        state = self._tuple_to_state(state_tuple, None)
        return self.pos_actions_from_state(state)
    
    def get_next_states_probs(self, state, action):
        # Returns a list of (next_state, probability) pairs
        # Note: the helper treats all negative battery levels as -1

        self.state = state

        # Handle reset and terminate actions

        if action == ("reset",):
            next_state = deepcopy(self.initial_state)
            next_state["turns_to_go"] -= 1
            return [(next_state, 1.0)]
        
        if action == ("terminate",):
            return []
        
        # Determine next robot location
        
        if action[0] == "move":
            next_rob_loc = action[1]
        else:
            next_rob_loc = state["robot"]["location"]
                
        # Decrease turns to go if applicable
        
        turns_to_go = state.get("turns_to_go", None)
        if turns_to_go is not None:
            turns_to_go -= 1

        # Determine next robot battery level
        rob = state["robot"]
        battery = rob["battery"]
        if action[0] == "move":
            battery -= self._move_cost(next_rob_loc)
        elif action == ("charge",):
            cs = self._charger_at(rob["location"])
            battery = min(
                battery + int(cs["charge_amount"]),
                int(rob["maximum_moves_left_possible"]),
            )
        elif action == ("emergency_charge",):
            battery = int(rob["maximum_moves_left_possible"])
        # else: battery remains the same for wait, pick-up


        # Calculate possible next pickup locations and their probabilities

        next_state_pickup_locations = []
        for p in state.get("pick_up_locations", []):
            move_prob = p.get("move_probability", 0.0)
            possible_locs = p.get("possible_locations", [])
            if move_prob <= 0.0 or len(possible_locs) <= 1:
                next_state_pickup_locations.append([({**p}, 1.0)])
            else:
                next_state_pickup_locations_temp = []
                for loc in possible_locs:
                    prob = move_prob / len(possible_locs) if loc != p["location"] else 1.0 - move_prob + move_prob / len(possible_locs)
                    new_p = {**p, "location": loc}
                    next_state_pickup_locations_temp.append((new_p, prob))
                next_state_pickup_locations.append(next_state_pickup_locations_temp)
        
        next_state_pickup_locations_combinations = list(itertools.product(*next_state_pickup_locations))

        all_combinations_next_state_pickups_and_probs = []
        for combination in next_state_pickup_locations_combinations:
            cur_prob = 1.0
            cur_state = []
            for item in combination:
                cur_state.append(item[0])
                cur_prob *= item[1]
            all_combinations_next_state_pickups_and_probs.append((cur_state, cur_prob))

        # Calculate possible next robovac positions and their probabilities
        
        next_state_robovacs = []
        for rv in state.get("robovacs", []):
            path = rv["path"]
            if len(path) <= 1:
                next_state_robovacs.append([({**rv}, 1.0)])
            else:
                next_state_robovacs_temp = []
                idx = int(rv["index"])
                direction = rv.get("direction", "F")

                possible_next_indices = []
                if idx == 0:
                    possible_next_indices = [(0, 0.25), (1, 0.75)]
                elif idx == len(path) - 1:
                    possible_next_indices = [(idx, 0.25), (idx - 1, 0.75)]
                else:
                    if direction == "F":
                        possible_next_indices = [(idx - 1, 0.25), (idx, 0.25), (idx + 1, 0.50)]
                    else:
                        possible_next_indices = [(idx + 1, 0.25), (idx, 0.25), (idx - 1, 0.50)]

                for next_idx, prob in possible_next_indices:
                    new_direction = "F" if next_idx == 0 else ("B" if next_idx == len(path) - 1 else direction)
                    new_rv = {**rv, "index": next_idx, "direction": new_direction}
                    next_state_robovacs_temp.append((new_rv, prob))

                next_state_robovacs.append(next_state_robovacs_temp)

        next_state_robovacs_combinations = list(itertools.product(*next_state_robovacs))

        # Combine pickup and robovac next states and probabilities

        all_combinations_next_state_robovacs_and_probs = []
        for combination in next_state_robovacs_combinations:
            cur_prob = 1.0
            cur_state = []
            for item in combination:
                cur_state.append(item[0])
                cur_prob *= item[1]
            all_combinations_next_state_robovacs_and_probs.append((cur_state, cur_prob))
        
        # Combine all combinations to form next random states

        next_random_state_prob = []

        for pickup_loc, pickup_prob in all_combinations_next_state_pickups_and_probs:
            for robovac_loc, robovac_prob in all_combinations_next_state_robovacs_and_probs:
                total_prob = pickup_prob * robovac_prob
                next_random_state_prob.append((pickup_loc, robovac_loc, total_prob))
        
        # Create the full next states list and combine with their probabilities

        next_pos_states_and_probs = []

        for pickup_locs, robovac_locs, prob in next_random_state_prob:
            next_state = deepcopy(state)
            next_state["robot"]["location"] = next_rob_loc
            next_state["robot"]["battery"] = battery
            next_state["pick_up_locations"] = pickup_locs
            next_state["robovacs"] = robovac_locs
            if turns_to_go is not None:
                next_state["turns_to_go"] = turns_to_go
            # Apply robovac collision battery damage
            dmg = int(next_state["robot"].get("robovac_battery_damage", 0))
            robot_loc = next_state["robot"]["location"]
            hits = 0
            for rv in next_state.get("robovacs", []):
                if self._robovac_loc(rv) == robot_loc:
                    hits += 1
            if hits:
                next_state["robot"]["battery"] -= hits * dmg
            # Reduce charger cooldowns
            if action == ("charge",):
                cs = self._charger_at(next_state["robot"]["location"])
                if cs is not None:
                    cs["cooldown"] = int(cs.get("charge_wait", 0))
            for cs in next_state.get("charging_stations", []):
                cs["cooldown"] = max(0, int(cs.get("cooldown", 0)) - 1)
            if next_state["robot"]["battery"] < -1:
                next_state["robot"]["battery"] = -1
            next_pos_states_and_probs.append((next_state, prob))

        return next_pos_states_and_probs
    
    def get_next_tuple_states_probs(self, state_tuple, action, turns_to_go):
        # Converts state tuple to state dictionary, then calls get_next_states_probs
        state = self._tuple_to_state(state_tuple, turns_to_go)
        next_states_probs = self.get_next_states_probs(state, action)
        next_tuples_probs = []
        for next_state, prob in next_states_probs:
            next_tuple = self._state_to_tuple(next_state)
            next_tuples_probs.append((next_tuple, prob))
        return next_tuples_probs

    def get_reward(self, action):
        # Returns the reward for taking the given action in the current state
        if action == ("reset",):
            return -RESET_PENALTY
        if action == ("emergency_charge",):
            return -EMERGENCY_CHARGE_PENALTY
        if action[0] == "pick-up":
            return PICK_UP_REWARD
        return 0
    
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
    def is_action_legal(self, state, action):
        robot = state[K_ROBOT]
        robot_loc = robot[K_ROB_LOC]
        battery = robot[K_ROB_BAT]
        infinite = state.get("infinite", False)
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
        if action != "reset":
            self.environment_step()
        self.check_collision_with_robovacs()

    def apply(self, action):
        if action == "reset":
            self.reset_environment()
            return
        if action == "terminate":
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
                rv["direction"] = "F"
            elif idx == len(path) - 1:
                rv["index"] = idx if random.random() < 0.25 else idx - 1
                rv["direction"] = "B"
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

class EndOfGame(Exception):
    pass
