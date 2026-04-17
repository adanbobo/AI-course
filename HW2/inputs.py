# inputs.py

finite_inputs = [
{
    "optimal": False,
    "turns_to_go": 8,
    "map": [
        ["P", "P", "P"],
        ["P", "P", "P"],
        ["P", "P", "P"],
    ],
    "robot": {
        "start": (2, 2),
        "starting_moves_left": 6,
        "maximum_moves_left_possible": 6,
        "robovac_battery_damage": 2,
        "uneven_floor_penalty": 2,
    },
    "pick_up_locations": [
        {"name": "math_exams", "location": (0, 0), "move_probability": 0.1,
            "possible_locations": [(0, 0), (1, 1), (2, 2)]},
        {"name": "english_exam", "location": (2, 1), "move_probability": 0.1,
            "possible_locations": [(2, 1), (1, 0)]},
    ],
    "robovacs": [
        {"name": "rv1", "index": 0, "direction": "F", "path": [(1, 1), (2, 1), (2, 2)]}
    ],
    "charging_stations": [{"location": (0, 0), "charge_amount": 5, "charge_wait": 1}],
    "uneven_floor": [(1, 2)],
},
{
    "optimal": False,
    "turns_to_go": 10,
    "map": [
        ["P", "P", "P", "P"],
        ["P", "I", "P", "P"],
        ["P", "P", "P", "P"],
        ["P", "P", "P", "P"],
    ],
    "robot": {
        "start": (3, 0),
        "starting_moves_left": 3,
        "maximum_moves_left_possible": 6,
        "robovac_battery_damage": 3,
        "uneven_floor_penalty": 2,
    },
    "pick_up_locations": [
        {"name": "m", "location": (2, 0), "move_probability": 0.0, "possible_locations": [(2, 0)]},
        {"name": "e", "location": (0, 2), "move_probability": 0.0, "possible_locations": [(0, 2)]},
    ],
    "robovacs": [
        {"name": "rv1", "index": 0, "direction": "F", "path": [(1, 0), (2, 0)]},
        {"name": "rv2", "index": 0, "direction": "F", "path": [(0, 3), (1, 3), (2, 3)]},
    ],
    "charging_stations": [{"location": (3, 0), "charge_amount": 4, "charge_wait": 2},
                            {"location": (0, 0), "charge_amount": 3, "charge_wait": 1}],
    "uneven_floor": [(2, 0)],
},
]
optimal_inputs = [
{
    "optimal": True,
    "turns_to_go": 50,
    "map": [
        ["P", "P", "P"],
        ["P", "I", "P"],
        ["P", "P", "P"],
    ],
    "robot": {
        "start": (2, 0),
        "starting_moves_left": 3,
        "maximum_moves_left_possible": 6,
        "robovac_battery_damage": 2,
        "uneven_floor_penalty": 2,
    },
    "pick_up_locations": [
        {"name": "a", "location": (0, 2), "move_probability": 0.0, "possible_locations": [(0, 2)]},
        {"name": "b", "location": (2, 2), "move_probability": 0.0, "possible_locations": [(2, 2)]},
    ],
    "robovacs": [
        {"name": "rv1", "index": 0, "direction": "F", "path": [(1, 2), (2, 2)]}
    ],
    "charging_stations": [{"location": (2, 0), "charge_amount": 4, "charge_wait": 2}],
    "uneven_floor": [(2, 1)],
},
]
pi_vi_inputs = [
{
    "optimal": True,
    "infinite": True,
    "gamma": 0.95,
    "map": [
        ["P", "P"],
        ["P", "P"],
    ],
    "robot": {
        "start": (1, 0),
        "starting_moves_left": 2,
        "maximum_moves_left_possible": 2,
        "robovac_battery_damage": 0,
        "uneven_floor_penalty": 2,
    },
    "pick_up_locations": [
        {"name": "p", "location": (0, 0), "move_probability": 0.0, "possible_locations": [(0, 0)]}
    ],
    "robovacs": [],
    "charging_stations": [{"location": (1, 0), "charge_amount": 2, "charge_wait": 1}],
    "uneven_floor": [],
},
{
    "optimal": True,
    "infinite": True,
    "gamma": 0.95,
    "map": [
        ["P", "P", "P"],
        ["P", "I", "P"],
    ],
    "robot": {
        "start": (1, 0),
        "starting_moves_left": 2,
        "maximum_moves_left_possible": 3,
        "robovac_battery_damage": 1,
        "uneven_floor_penalty": 2,
    },
    "pick_up_locations": [
        {"name": "p", "location": (0, 2), "move_probability": 0.0, "possible_locations": [(0, 2)]}
    ],
    "robovacs": [
        {"name": "rv", "index": 0, "direction": "F", "path": [(0, 1), (0, 2)]}
    ],
    "charging_stations": [{"location": (1, 0), "charge_amount": 2, "charge_wait": 1}],
    "uneven_floor": [],
},
]
