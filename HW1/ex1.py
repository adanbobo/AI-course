
import math
import heapq
import search

ids = ["325468510", "213191083"]


class RobotNavigationProblem(search.Problem):

    def __init__(self, data):
        # --- load map and robot info ---
        self._load_map(data["map"])
        self._load_robot(data["robot"])

        # --- environment attributes ---
        self.uneven = {tuple(p) for p in data.get("uneven_floor", [])}
        self.blocked_tiles = set()
        self.destination = None
        self._scan_map_for_special_tiles()

        # --- stations ---
        self.station_list = []
        self.station_index = {}
        self.stations = {}
        self._load_stations(data.get("charging_stations", []))

        # --- robovacs ---
        self.robovacs = []
        self.cycle = self._compute_cycle(data.get("robovacs", {}))
        self.robovac_occupancy = self._precompute_robovac_positions()

        # --- precomputed backward energy (Dijkstra) ---
        self.min_battery_map = self.reverse_dijkstra_energy(self.destination)

        # --- initial state for search ---
        init_cool = tuple(0 for _ in self.station_list)
        r0, c0 = self.start_loc
        initial_state = (r0, c0, self.start_battery, 0, init_cool)
        super().__init__(initial_state)

    # -----------------------------
    # INTERNAL LOADING HELPERS
    # -----------------------------
    def _load_map(self, raw_map):
        self.map_data = raw_map
        self.rows = len(raw_map)
        self.cols = len(raw_map[0])

    def _load_robot(self, bot):
        self.start_loc = tuple(bot["starting_location"])
        self.start_battery = bot["starting_moves_left"]
        self.max_battery = bot.get("maximum_moves_left_possible", self.start_battery)
        self.robovac_damage = bot.get("robovac_battery_damage", 0)
        self.uneven_penalty = bot.get("uneven_floor_penalty", 0)

    def _scan_map_for_special_tiles(self):
        for r, row in enumerate(self.map_data):
            for c, cell in enumerate(row):
                pos = (r, c)
                if cell == "I":
                    self.blocked_tiles.add(pos)
                elif cell == "D":
                    self.destination = pos

        if self.destination is None:
            self.destination = (-1, -1)

    def _load_stations(self, raw):
        for station in raw:
            loc = tuple(station["location"])
            idx = len(self.station_list)
            self.station_list.append(loc)
            self.station_index[loc] = idx
            self.stations[loc] = (
                station["charge_amount"],
                station["charge_wait"]
            )

    # -----------------------------
    # ROBOVAC UTILITIES
    # -----------------------------
    def _compute_cycle(self, rv_dict):
        """Build robovac paths and compute the global cycle length."""
        cycle = 1
        for path in rv_dict.values():
            seq = [tuple(p) for p in path]
            if not seq:
                continue

            L = len(seq)
            per = 1 if L == 1 else 2 * (L - 1)
            self.robovacs.append((seq, per))

            cycle = (cycle * per) // math.gcd(cycle, per)

        return cycle if self.robovacs else 1

    def _precompute_robovac_positions(self):
        occupancy = [dict() for _ in range(self.cycle)]
        for t in range(self.cycle):
            cell_counts = occupancy[t]
            for path, per in self.robovacs:
                pos = self._ping(path, per, t)
                cell_counts[pos] = cell_counts.get(pos, 0) + 1
        return occupancy

    @staticmethod
    def _ping(path, per, t):
        L = len(path)
        if L == 1:
            return path[0]
        k = t % per
        return path[k] if k < L else path[per - k]

    # -----------------------------
    # ENERGY MODEL
    # -----------------------------
    def move_cost(self, src, dst):
        if src not in self.uneven and dst not in self.uneven:
            return 1
        return self.uneven_penalty

    # -----------------------------
    # MAP TRAVERSAL
    # -----------------------------
    def _valid_step(self, r, c):
        return (
                0 <= r < self.rows and
                0 <= c < self.cols and
                (r, c) not in self.blocked_tiles
        )

    # -----------------------------
    # DIJKSTRA ENERGY REVERSE SEARCH
    # -----------------------------
    def reverse_dijkstra_energy(self, destination):
        dist = {destination: 0}
        pq = [(0, destination)]
        directions = ((1, 0), (-1, 0), (0, 1), (0, -1))

        while pq:
            d, (r, c) = heapq.heappop(pq)
            if d != dist[(r, c)]:
                continue

            for dr, dc in directions:
                nr, nc = r + dr, c + dc
                if not self._valid_step(nr, nc):
                    continue

                cost = self.move_cost((r, c), (nr, nc))
                new_d = d + cost

                if new_d < dist.get((nr, nc), float("inf")):
                    dist[(nr, nc)] = new_d
                    heapq.heappush(pq, (new_d, (nr, nc)))

        return dist

    def actions(self, state):

        row, col, battery, time_mod, cooldowns = state

        if (row, col) == (-1, -1):
            return []

        if battery < 0:
            return []

        if battery == 0 and ((row, col) not in self.stations and (row, col) != self.destination):
            return []

        possible = []

        if (row, col) == self.destination:
            possible.append("finish")

        if (row, col) in self.station_index:
            st_id = self.station_index[(row, col)]
            if cooldowns[st_id] == 0:
                possible.append("charge")

        if battery > 0:
            moves = [(0, 1), (0, -1), (1, 0), (-1, 0)]

            for dr, dc in moves:
                nr, nc = row + dr, col + dc

                if not self._valid_step(nr, nc):
                    continue

                step_cost = self.move_cost((row, col), (nr, nc))
                if battery >= step_cost:
                    possible.append(("move", (nr, nc)))

        possible.append("wait")

        return possible

    def result(self, state, action):
        r, c, battery, tmod, cooldowns = state

        # פעולה מיוחדת: סיום – אין שינוי בזמן ובקירורים
        if action == "finish":
            return (-1, -1, battery, tmod, cooldowns)

        # עדכון זמן ומצבי קירור
        next_time = (tmod + 1) % self.cycle
        next_cooldowns = []
        for cd in cooldowns:
            if cd > 0:
                next_cooldowns.append(cd - 1)
            else:
                next_cooldowns.append(0)

        # ערכי ברירת מחדל – נשארים במקום, אותה בטרייה
        new_r, new_c = r, c
        new_bat = battery
        loc = (r, c)

        # טיפול בפעולה
        if action == "charge":
            charge_amt, wait = self.stations[loc]
            tmp_bat = battery + charge_amt
            if tmp_bat > self.max_battery:
                tmp_bat = self.max_battery
            new_bat = tmp_bat

            sid = self.station_index[loc]
            next_cooldowns[sid] = wait

        elif action == "wait":
            # לא עושים כלום, רק הזמן/קירורים כבר עודכנו
            pass

        elif isinstance(action, tuple) and action[0] == "move":
            _, (nr, nc) = action
            move_cost = self.move_cost((r, c), (nr, nc))
            new_r, new_c = nr, nc
            new_bat = battery - move_cost

        # פגיעה מרובובאקים אם יש בטרייה והם קיימים
        if self.robovacs and new_bat >= 0:
            robovacs_positions = self.robovac_occupancy[next_time]
            hits = robovacs_positions.get((new_r, new_c), 0)
            if hits > 0:
                new_bat = new_bat - hits * self.robovac_damage

        # טיפול במצבי בטרייה לא חוקיים
        if new_bat < 0:
            new_bat = -1
        elif new_bat == 0:
            new_loc = (new_r, new_c)
            if new_loc not in self.stations and new_loc != self.destination:
                new_bat = -1

        return (new_r, new_c, new_bat, next_time, tuple(next_cooldowns))

    def goal_test(self, state):
        r, c, *_ = state
        return (r, c) == (-1, -1)

    def h(self, node):
        r, c, battery, _, _ = node.state

        if (r, c) == (-1, -1):
            return 0

        if self.destination is None or self.destination == (-1, -1):
            return 0

        gx, gy = self.destination

        return abs(gx - r) + abs(gy - c)

def create_robot_navigation_problem(desc):
    return RobotNavigationProblem(desc)


def astar_search(problem, heuristic):
    BATTERY_PREFERENCE = 0.002
    H_WEIGHT = 1.3

    def evaluate(node):
        row, col, bat, t, cds = node.state

        return (
            node.path_cost
            + H_WEIGHT * heuristic(node)
            - BATTERY_PREFERENCE * max(0, bat)
        )

    frontier = search.PriorityQueue(order=min, f=evaluate)
    start = search.Node(problem.initial)
    frontier.append(start)

    visited = {problem.initial: 0}

    while frontier:
        node = frontier.pop()

        if node.path_cost > visited.get(node.state, float("inf")):
            continue

        if problem.goal_test(node.state):
            return node

        for succ in node.expand(problem):
            g = succ.path_cost

            if g < visited.get(succ.state, float("inf")):
                visited[succ.state] = g
                frontier.append(succ)

    return None
