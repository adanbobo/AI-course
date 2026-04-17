import ex31 as ex3
import time
from inputs import non_comp_problems

TIMEOUT_DURATION = 60

def timeout_exec(func, args=(), kwargs={}, timeout_duration=10, default=None):
    """This function will spawn a thread and run the given function
    using the args, kwargs and return the given default value if the
    timeout_duration is exceeded.
    """
    import threading

    class InterruptableThread(threading.Thread):
        def __init__(self):
            threading.Thread.__init__(self)
            self.result = default

        def run(self):
            self.result = func(*args, **kwargs)

    it = InterruptableThread()
    it.start()
    it.join(timeout_duration)
    if it.is_alive():
        return default
    else:
        return it.result



def solve_problem(p, translator_to_CNF, SAT_solver, translator_from_assignment, timeout):
    t0 = time.time()

    try:
        res = timeout_exec(
            translator_to_CNF, [p], timeout_duration=timeout
        )
        if res is None:
            return None, None, None, -2
        variables, CNF_formula = res

        t1 = time.time()
        res = timeout_exec(
            SAT_solver,
            [variables, CNF_formula, {}],
            timeout_duration=timeout - (t1 - t0)
        )
        if res is None:
            return None, None, None, -2
        is_satisfiable, assignment = res

        t2 = time.time()
        if is_satisfiable:
            res = timeout_exec(
                translator_from_assignment,
                [variables, assignment, p],
                timeout_duration=timeout - (t2 - t0)
            )
            if res is None:
                return None, None, None, -2
            board = res
        else:
            board = [[]]

        return is_satisfiable, assignment, board, time.time() - t0

    except Exception:
        return None, None, None, -3



def solve_problems(problems):
    solved = 0
    for p in problems:
        timeout = TIMEOUT_DURATION
        solvable, assignment, board, time = solve_problem(p, ex3.to_CNF, ex3.solve_SAT, ex3.numbers_assignment, timeout)
        if solvable is None:
            print("The code didn't run correctly")
            continue
        if not solvable:
            print("Your code claims the problem is not solvable")
            continue
        print("Your code claims the problem is solvable")
        is_correct, string_exlanation = is_valid_solution(p, board)
        if is_correct:
            print("Your solution is Correct!")
            continue
        print("Your solution is not correct")
        print(string_exlanation)
        continue
        


def is_valid_solution(input, board):
    """
    task: (rect_size, initial_clues, sum_cages)
        rect_size: (h, w) e.g., (2, 4)
        initial_clues: list of (row, col, val)
        sum_cages: list of (r1, c1, r2, c2, target_sum)
    solution: 2D list (matrix) of values
    """
    (rect_h, rect_w) = input[0]
    initial_clues = input[1]
    sum_cages = input[2]
    
    n = len(board)  # Total board size (e.g., 8)
    valid_digits = set(range(1, n + 1))

    # 1. Check Row Constraints
    for r in range(n):
        if set(board[r]) != valid_digits:
            return False, f"Row {r} is invalid."

    # 2. Check Column Constraints
    for c in range(n):
        col_values = [board[r][c] for r in range(n)]
        if set(col_values) != valid_digits:
            return False, f"Column {c} is invalid."

    # 3. Check Rectangle (Box) Constraints
    for r_offset in range(0, n, rect_h):
        for c_offset in range(0, n, rect_w):
            box_values = []
            for r in range(rect_h):
                for c in range(rect_w):
                    box_values.append(board[r_offset + r][c_offset + c])
            if set(box_values) != valid_digits:
                return False, f"Box starting at ({r_offset}, {c_offset}) is invalid."

    # 4. Check Initial Assignments (Clues)
    for r, c, val in initial_clues:
        if board[r][c] != val:
            return False, f"Clue at ({r}, {c}) mismatched. Expected {val}, got {board[r][c]}."

    # 5. Check Sum Cages
    for r1, c1, r2, c2, target_sum in sum_cages:
        # Calculate bounding box sum (assuming rectangular cages based on your 5-tuple format)
        current_sum = 0
        for r in range(min(r1, r2), max(r1, r2) + 1):
            for c in range(min(c1, c2), max(c1, c2) + 1):
                current_sum += board[r][c]
        
        if current_sum != target_sum:
            return False, f"Cage sum mismatch: expected {target_sum}, got {current_sum}."

    return True, "Solution is valid!"
        
                


def main():
    print(ex3.ids)
    """Here goes the input you want to check"""

    solve_problems(non_comp_problems)


if __name__ == '__main__':
    main()
