ids = ["212703029", "325468510"]

from helper import HelperClass
import random
import math
import time

class OptimalRobotAgent:
    def __init__(self, initial):
        self.initial = initial
        self.helper = HelperClass(initial)
        self.states = self.helper.calculate_all_state_tuples()
        self.policies = [] # policies[k] for k turns to go
        self.compute_policy()

    def compute_policy(self):
        turns = self.initial.get("turns_to_go", 0)
        # V[s] = max expected reward with k turns to go
        V_prev = {s: 0.0 for s in self.states}
        self.policies.append({}) # k=0

        for k in range(1, turns + 1):
            V_curr = {}
            policy_k = {}
            for s in self.states:
                actions = self.helper.pos_actions_from_state_tuple(s)
                if not actions:
                    V_curr[s] = 0.0
                    policy_k[s] = ("wait",)
                    continue

                best_val = -float('inf')
                best_act = actions[0]
                
                for a in actions:
                    reward = self.helper.get_reward(a)
                    # We pass 0 as dummy turns_to_go; helper uses it only for state dict creation
                    transitions = self.helper.get_next_tuple_states_probs(s, a, 0)
                    
                    future_val = 0.0
                    for next_s, prob in transitions:
                        future_val += prob * V_prev.get(next_s, 0.0)
                    
                    total_val = reward + future_val
                    if total_val > best_val:
                        best_val = total_val
                        best_act = a
                
                V_curr[s] = best_val
                policy_k[s] = best_act
            V_prev = V_curr
            self.policies.append(policy_k)

    def act(self, state):
        s_tuple = self.helper._state_to_tuple(state)
        turns_left = state.get("turns_to_go", 0)
        turns_idx = min(turns_left, len(self.policies) - 1)
        if turns_idx < 1: return ("wait",)
        return self.policies[turns_idx].get(s_tuple, ("wait",))


class RobotAgent:
    def __init__(self, initial):
        self.initial = initial
        self.helper = HelperClass(initial)
        self.map = initial["map"]
        self.rows = len(self.map)
        self.cols = len(self.map[0])
        self.time_limit = 4.8
        
        # Danger zones for heuristic fallback
        self.danger_zones = set()
        for rv in initial.get("robovacs", []):
            for loc in rv["path"]: self.danger_zones.add(loc)

        # Hybrid check: Use VI for small state spaces, MCTS for large ones
        # Estimate state space size
        try:
            self.states = self.helper.calculate_all_state_tuples()
            if len(self.states) < 15000:
                self.use_vi = True
                self.vi_agent = OptimalRobotAgent(initial)
            else:
                self.use_vi = False
        except:
            self.use_vi = False

    def act(self, state):
        if self.use_vi:
            return self.vi_agent.act(state)
        
        # MCTS Fallback for large maps
        start_time = time.time()
        robot = state["robot"]
        if robot["battery"] < 0: return ("emergency_charge",)

        actions = self.helper.pos_actions_from_state(state)
        if not actions: return ("wait",)
        viable = [a for a in actions if a[0] not in ("reset", "terminate")]
        if not viable: return actions[0]

        # Heuristic Override: If can pick up, do it immediately
        for a in viable:
            if a[0] == "pick-up": return a

        horizon = min(10, state["turns_to_go"])
        scores = {a: 0.0 for a in viable}
        counts = {a: 0 for a in viable}
        
        while time.time() - start_time < self.time_limit:
            for a in viable:
                if time.time() - start_time >= self.time_limit: break
                scores[a] += self.simulate(state, a, horizon)
                counts[a] += 1

        return max(viable, key=lambda a: scores[a]/counts[a] if counts[a]>0 else -float('inf'))

    def simulate(self, start_state, first_action, horizon):
        curr = {
            "robot": start_state["robot"].copy(),
            "robovacs": [r.copy() for r in start_state.get("robovacs", [])],
            "pick_up_locations": [p.copy() for p in start_state.get("pick_up_locations", [])],
            "charging_stations": [c.copy() for c in start_state.get("charging_stations", [])],
            "turns_to_go": start_state["turns_to_go"],
            "map": self.map,
            "uneven_floor": start_state.get("uneven_floor", [])
        }
        total = self.apply_sim_step(curr, first_action)
        for _ in range(horizon - 1):
            if curr["turns_to_go"] <= 0: break
            total += self.apply_sim_step(curr, self.default_policy(curr))
        return total

    def default_policy(self, state):
        robot = state["robot"]
        loc = robot["location"]
        for p in state["pick_up_locations"]:
            if p["location"] == loc: return ("pick-up", p["name"], loc)
        if robot["battery"] < 0: return ("emergency_charge",)
        for cs in state["charging_stations"]:
            if cs["location"] == loc and cs["cooldown"] == 0 and robot["battery"] < robot["maximum_moves_left_possible"]:
                 return ("charge",)
        
        moves = []
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = loc[0]+dx, loc[1]+dy
            if 0 <= nx < self.rows and 0 <= ny < self.cols and self.map[nx][ny] != "I":
                 moves.append(("move", (nx, ny)))
        if not moves: return ("wait",)

        target = None
        min_dist = float('inf')
        if robot["battery"] < int(robot["maximum_moves_left_possible"]) / 3:
            for cs in state["charging_stations"]:
                d = abs(cs["location"][0]-loc[0]) + abs(cs["location"][1]-loc[1])
                if d < min_dist: min_dist, target = d, cs["location"]
        
        if target is None:
            for p in state["pick_up_locations"]:
                d = abs(p["location"][0]-loc[0]) + abs(p["location"][1]-loc[1])
                if p["location"] in self.danger_zones: d += 10
                if d < min_dist: min_dist, target = d, p["location"]
        
        if target:
            random.shuffle(moves)
            best_m, best_d = moves[0], float('inf')
            for m in moves:
                cost = int(robot.get("uneven_floor_penalty", 1)) if (m[1] in state["uneven_floor"] or loc in state["uneven_floor"]) else 1
                if robot["battery"] - cost < 0: continue
                d = abs(m[1][0]-target[0]) + abs(m[1][1]-target[1])
                if m[1] in self.danger_zones: d += 10
                if d < best_d: best_d, best_m = d, m
            return best_m
        return random.choice(moves)

    def apply_sim_step(self, state, action):
        reward = 0
        robot = state["robot"]
        if action == ("reset",):
            reward -= 2
            state["turns_to_go"] -= 1
            return reward
        if action[0] == "move":
            cost = int(robot.get("uneven_floor_penalty", 1)) if (action[1] in state["uneven_floor"] or robot["location"] in state["uneven_floor"]) else 1
            robot["battery"] -= cost
            robot["location"] = action[1]
        elif action[0] == "pick-up": reward += 2
        elif action == ("charge",):
            for cs in state["charging_stations"]:
                if cs["location"] == robot["location"]:
                    robot["battery"] = min(robot["battery"] + cs["charge_amount"], robot["maximum_moves_left_possible"])
                    cs["cooldown"] = cs["charge_wait"]
                    break
        elif action == ("emergency_charge",):
            reward -= 5
            robot["battery"] = int(robot["maximum_moves_left_possible"])

        # Env
        for p in state["pick_up_locations"]:
            if random.random() < p["move_probability"]: p["location"] = random.choice(p["possible_locations"])
        for rv in state["robovacs"]:
            path = rv["path"]
            if len(path) > 1:
                idx, dr, r = rv["index"], rv["direction"], random.random()
                if idx == 0: rv["index"], rv["direction"] = (0 if r < 0.25 else 1), "F"
                elif idx == len(path)-1: rv["index"], rv["direction"] = (idx if r < 0.25 else idx-1), "B"
                else:
                    if dr == "F": rv["index"] = idx-1 if r<0.25 else (idx if r<0.5 else idx+1)
                    else: rv["index"] = idx+1 if r<0.25 else (idx if r<0.5 else idx-1)
                    if rv["index"] == 0: rv["direction"] = "F"
                    elif rv["index"] == len(path)-1: rv["direction"] = "B"
        for rv in state["robovacs"]:
            if rv["path"][rv["index"]] == robot["location"]: robot["battery"] -= robot.get("robovac_battery_damage", 0)
        for cs in state["charging_stations"]:
            if cs["cooldown"] > 0 and action != ("charge",): cs["cooldown"] -= 1
        state["turns_to_go"] -= 1
        return reward


class InfiniteRobotAgent:
    def __init__(self, initial, gamma):
        self.initial = initial
        self.gamma = gamma
        self.helper = HelperClass(initial)
        self.states = self.helper.calculate_all_state_tuples()
        self.V = {s: 0.0 for s in self.states}
        self.policy = {s: ("wait",) for s in self.states}
        self.compute_policy()

    def compute_policy(self):
        epsilon = 1e-4
        while True:
            delta = 0
            new_V = {}
            for s in self.states:
                actions = self.helper.pos_actions_from_state_tuple(s)
                if not actions:
                    new_V[s] = 0.0
                    continue
                best_val = -float('inf')
                best_act = actions[0]
                for a in actions:
                    reward = self.helper.get_reward(a)
                    transitions = self.helper.get_next_tuple_states_probs(s, a, 0)
                    future = sum(prob * self.V.get(next_s, 0.0) for next_s, prob in transitions)
                    val = reward + self.gamma * future
                    if val > best_val:
                        best_val, best_act = val, a
                new_V[s] = best_val
                self.policy[s] = best_act
                delta = max(delta, abs(new_V[s] - self.V[s]))
            self.V = new_V
            if delta < epsilon: break

    def act(self, state):
        return self.policy.get(self.helper._state_to_tuple(state), ("wait",))

    def value(self, state):
        return self.V.get(self.helper._state_to_tuple(state), 0.0)

class ExampleRobotAgent:
    def __init__(self, initial): self.map = initial["map"]
    def act(self, state):
        loc = state["robot"]["location"]
        new_loc = (loc[0] - 1, loc[1])
        if 0 <= new_loc[0] < len(self.map) and 0 <= new_loc[1] < len(self.map[0]) and self.map[new_loc[0]][new_loc[1]] != "I":
            return ("move", new_loc)
        return ("wait",)
