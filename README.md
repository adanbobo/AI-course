# AI-course
# AI Course Projects

This repository contains my assignments for the Artificial Intelligence course.

## HW1 – Deterministic Search

In this assignment, I implemented a robot navigation problem using search algorithms.
**Grade:** 92/100

### Main concepts:
- State representation
- Action modeling
- A* search algorithm
- Heuristic design
- Grid-based environment

### Goal:
Guide a robot to a destination while minimizing the number of turns, considering:
- Battery constraints
- Charging stations
- Moving obstacles (robovacs)
- Uneven terrain

### Files:
- ex1.py – main implementation
- check.py – runner for testing
- search.py – search algorithms
- utils.py – helper functions
- problems.py – input scenarios
- HW1.pdf – assignment description


## HW2 – Stochastic Task

In this assignment, I implemented decision-making under uncertainty using stochastic environments.

### Main concepts:
- Stochastic modeling
- Markov Decision Processes (MDP)
- Value iteration / policy optimization
- Expected rewards maximization

### Goal:
Maximize the total reward by collecting exams in a dynamic and uncertain environment, considering:
- Random movement of robovacs
- Changing pickup locations
- Limited number of turns
- Trade-offs between actions (reset, emergency charge, etc.)

### Evaluation
**Grade:** 105/100 
Achieved high performance in both optimal and stochastic decision-making tasks.

 ### Files:
- ex2.py – core implementation of decision-making agents for stochastic environments
- check.py – execution engine that simulates the environment and evaluates actions
- helper.py – provides state transformations, action generation, and transition probabilities
- inputs.py – collection of test cases defining different environments
- utils.py – auxiliary helper functions supporting the implementation


