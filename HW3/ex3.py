
from typing import Tuple, List, Any, Optional, Dict
from collections import defaultdict, Counter

ids = ["325468510", "213690928"]

Lit = Tuple[Any, bool]          # (var, is_positive)
Clause = List[Lit]
CNF = List[Clause]
Assignment = Dict[Any, bool]


# ============================================================
# Helpers (allowed to add helper functions)
# ============================================================
def _v(r: int, c: int, val: int) -> Tuple[int, int, int]:
    """Decision variable: cell (r,c) has value val (1..N)."""
    return (r, c, val)


def _add_clause(cnf: CNF, clause: Clause) -> None:
    cnf.append(list(clause))


def _at_most_one_sequential(cnf: CNF, xs: List[Any], variables_out: List[Any], ctx: Any) -> None:
    """Sinz sequential encoding for At-Most-One(xs) in O(n)."""
    n = len(xs)
    if n <= 1:
        return
    s = [("seq", ctx, i) for i in range(n - 1)]
    variables_out.extend(s)

    # (¬x1 ∨ s1)
    _add_clause(cnf, [(xs[0], False), (s[0], True)])

    for i in range(1, n - 1):
        # (¬xi ∨ si)
        _add_clause(cnf, [(xs[i], False), (s[i], True)])
        # (¬s_{i-1} ∨ si)
        _add_clause(cnf, [(s[i - 1], False), (s[i], True)])
        # (¬xi ∨ ¬s_{i-1})
        _add_clause(cnf, [(xs[i], False), (s[i - 1], False)])

    # (¬xn ∨ ¬s_{n-1})
    _add_clause(cnf, [(xs[n - 1], False), (s[n - 2], False)])


def _exactly_one(cnf: CNF, xs: List[Any], variables_out: List[Any], ctx: Any) -> None:
    """Exactly-One(xs) = At-Least-One + At-Most-One(sequential)."""
    _add_clause(cnf, [(x, True) for x in xs])  # at least one
    _at_most_one_sequential(cnf, xs, variables_out, ctx)


def _lit_eval(lit: Lit, asg: Assignment) -> Optional[bool]:
    """Return literal truth value under assignment: True/False/None (unassigned)."""
    v, pos = lit
    if v not in asg:
        return None
    return asg[v] if pos else (not asg[v])


# ============================================================
# 1) to_CNF
# ============================================================
def to_CNF(input) -> Tuple[list, List[List[Tuple[Any, bool]]]]:
    """
    input = [
      (K, L),
      [(r,c,val), ...],            # clues
      [(r1,c1,r2,c2,sum), ...]     # sum cages (bounding rectangle inclusive)
    ]
    Returns (variables, CNF_formula)
    """
    (K, L) = input[0]
    clues = input[1]
    cages = input[2]
    N = K * L

    cnf: CNF = []
    variables: List[Any] = []

    # base vars
    for r in range(N):
        for c in range(N):
            for val in range(1, N + 1):
                variables.append(_v(r, c, val))

    # cell: exactly one value
    for r in range(N):
        for c in range(N):
            xs = [_v(r, c, val) for val in range(1, N + 1)]
            _exactly_one(cnf, xs, variables, ("cell", r, c))

    # row: each val appears exactly once
    for r in range(N):
        for val in range(1, N + 1):
            xs = [_v(r, c, val) for c in range(N)]
            _exactly_one(cnf, xs, variables, ("row", r, val))

    # col: each val appears exactly once
    for c in range(N):
        for val in range(1, N + 1):
            xs = [_v(r, c, val) for r in range(N)]
            _exactly_one(cnf, xs, variables, ("col", c, val))

    # blocks KxL: each val appears exactly once
    for br in range(0, N, K):
        for bc in range(0, N, L):
            for val in range(1, N + 1):
                xs = [_v(br + dr, bc + dc, val) for dr in range(K) for dc in range(L)]
                _exactly_one(cnf, xs, variables, ("blk", br, bc, val))

    # clues
    for (r, c, val) in clues:
        _add_clause(cnf, [(_v(r, c, val), True)])

    # ----- sum cages: cache allowed tuples by (m, target_sum, N)
    allowed_cache: Dict[Tuple[int, int, int], List[Tuple[int, ...]]] = {}

    def allowed_tuples(m: int, target_sum: int) -> List[Tuple[int, ...]]:
        key = (m, target_sum, N)
        if key in allowed_cache:
            return allowed_cache[key]

        res: List[Tuple[int, ...]] = []

        def dfs(i: int, s: int, cur: List[int]) -> None:
            if i == m:
                if s == target_sum:
                    res.append(tuple(cur))
                return
            rem = m - i
            if s + rem * 1 > target_sum or s + rem * N < target_sum:
                return
            for v in range(1, N + 1):
                ns = s + v
                rem2 = m - (i + 1)
                if ns + rem2 * 1 > target_sum or ns + rem2 * N < target_sum:
                    continue
                cur.append(v)
                dfs(i + 1, ns, cur)
                cur.pop()

        dfs(0, 0, [])
        allowed_cache[key] = res
        return res

    for (r1, c1, r2, c2, target) in cages:
        r_lo, r_hi = min(r1, r2), max(r1, r2)
        c_lo, c_hi = min(c1, c2), max(c1, c2)
        cells = [(r, c) for r in range(r_lo, r_hi + 1) for c in range(c_lo, c_hi + 1)]
        m = len(cells)

        allowed = allowed_tuples(m, target)
        if not allowed:
            _add_clause(cnf, [])  # UNSAT
            continue

        cage_id = ("cage", r1, c1, r2, c2, target)
        aux = [(cage_id, i) for i in range(len(allowed))]
        variables.extend(aux)

        # choose one allowed tuple
        _add_clause(cnf, [(a, True) for a in aux])  # at least one
        _at_most_one_sequential(cnf, aux, variables, ("cage_amo", cage_id))  # at most one

        # aux_i -> (cell_j == allowed[i][j])
        for i, tup in enumerate(allowed):
            a = aux[i]
            for j, (r, c) in enumerate(cells):
                _add_clause(cnf, [(a, False), (_v(r, c, tup[j]), True)])

    return variables, cnf


# ============================================================
# 3) clause_status (general, independent of our encoding)
# ============================================================
def clause_status(
        clause: List[Tuple[Any, bool]],
        assignment: dict
) -> Tuple[str, Optional[Tuple[Any, bool]]]:
    """
    status in {"satisfied","unit","conflict","unresolved"}
    if unit: returns (var, value_to_set)
    """
    unassigned: List[Lit] = []
    for (v, pos) in clause:
        if v in assignment:
            lit_val = assignment[v] if pos else (not assignment[v])
            if lit_val:
                return "satisfied", None
        else:
            unassigned.append((v, pos))

    if len(unassigned) == 0:
        return "conflict", None
    if len(unassigned) == 1:
        v, pos = unassigned[0]
        return "unit", (v, True if pos else False)
    return "unresolved", None


# ============================================================
# 2) solve_SAT (DPLL + watched literals)
# ============================================================
def solve_SAT(
        variables: list,
        CNF_formula: List[List[Tuple[Any, bool]]],
        assignment: dict
) -> Tuple[bool, Optional[dict]]:
    """
    Returns (is_satisfiable, full_assignment_dict_or_None)
    """

    # quick UNSAT check
    for cl in CNF_formula:
        if len(cl) == 0:
            return False, None

    var_set = set(variables)
    asg: Assignment = dict(assignment)

    # heuristic frequency
    freq = Counter()
    for cl in CNF_formula:
        for (v, _) in cl:
            freq[v] += 1

    # watched literals
    watch_pos: List[Tuple[int, int]] = []
    watched_by: Dict[Lit, List[int]] = defaultdict(list)

    for ci, cl in enumerate(CNF_formula):
        if len(cl) == 1:
            i1 = i2 = 0
        else:
            i1, i2 = 0, 1
        watch_pos.append((i1, i2))
        watched_by[cl[i1]].append(ci)
        watched_by[cl[i2]].append(ci)

    prop_queue: List[Any] = []

    def enqueue(v: Any) -> None:
        prop_queue.append(v)

    # seed unit clauses
    for cl in CNF_formula:
        st, need = clause_status(cl, asg)
        if st == "conflict":
            return False, None
        if st == "unit" and need is not None:
            v, val = need
            if v in asg and asg[v] != val:
                return False, None
            if v not in asg:
                asg[v] = val
                enqueue(v)

    def choose_unassigned(a: Assignment) -> Any:
        best_v = None
        best_f = -1
        for v in var_set:
            if v not in a:
                f = freq.get(v, 0)
                if f > best_f:
                    best_f = f
                    best_v = v
        if best_v is None:
            for v in variables:
                if v not in a:
                    return v
        return best_v

    def propagate(a: Assignment) -> bool:
        while prop_queue:
            v = prop_queue.pop()

            # which literal becomes false due to v assignment?
            false_lits = [(v, False)] if a[v] is True else [(v, True)]

            for flit in false_lits:
                wlist = watched_by.get(flit, [])
                i = 0
                while i < len(wlist):
                    ci = wlist[i]
                    cl = CNF_formula[ci]
                    w1, w2 = watch_pos[ci]

                    if cl[w1] == flit:
                        bad_idx, other_idx = w1, w2
                    elif cl[w2] == flit:
                        bad_idx, other_idx = w2, w1
                    else:
                        i += 1
                        continue

                    other = cl[other_idx]
                    other_val = _lit_eval(other, a)
                    if other_val is True:
                        i += 1
                        continue

                    # find new watch
                    found_new = False
                    for k in range(len(cl)):
                        if k == other_idx:
                            continue
                        lk = cl[k]
                        valk = _lit_eval(lk, a)
                        if valk is not False:
                            # move watch
                            if bad_idx == w1:
                                watch_pos[ci] = (k, w2)
                            else:
                                watch_pos[ci] = (w1, k)

                            # remove from current watched list fast
                            wlist[i] = wlist[-1]
                            wlist.pop()
                            watched_by[lk].append(ci)
                            found_new = True
                            break

                    if found_new:
                        continue

                    # cannot move watch => unit or conflict
                    if other_val is False:
                        return False

                    ov, opos = other
                    need_val = True if opos else False
                    if ov in a:
                        if a[ov] != need_val:
                            return False
                    else:
                        a[ov] = need_val
                        enqueue(ov)
                    i += 1
        return True

    def all_satisfied(a: Assignment) -> bool:
        for cl in CNF_formula:
            st, _ = clause_status(cl, a)
            if st != "satisfied":
                return False
        return True

    def dpll(a: Assignment) -> Optional[Assignment]:
        if not propagate(a):
            return None
        if all_satisfied(a):
            # fill remaining vars
            for v in variables:
                if v not in a:
                    a[v] = False
            return a

        v = choose_unassigned(a)
        if v is None:
            return None

        for val in (True, False):
            a2 = dict(a)
            a2[v] = val
            enqueue(v)
            res = dpll(a2)
            prop_queue.clear()
            if res is not None:
                return res
        return None

    if not propagate(asg):
        return False, None

    res = dpll(dict(asg))
    if res is None:
        return False, None
    return True, res


# ============================================================
# 4) numbers_assignment
# ============================================================
def numbers_assignment(
        variables: list,
        assignment: dict,
        input: Any
) -> List[List[int]]:
    (K, L) = input[0]
    N = K * L
    clues = input[1]

    board = [[0 for _ in range(N)] for _ in range(N)]

    for r in range(N):
        for c in range(N):
            chosen = 0
            for val in range(1, N + 1):
                if assignment.get(_v(r, c, val), False):
                    chosen = val
                    break
            board[r][c] = chosen

    # enforce clues
    for (r, c, val) in clues:
        board[r][c] = val

    return board

def inspect_problem(p, out_prefix: str = "inspect"):
    """
    Run the whole pipeline on ONE problem and dump FULL outputs to files:
    - variables (all)
    - CNF (all clauses)
    - SAT result + assignment (all)
    - board
    """

    variables, cnf = to_CNF(p)

    # 1) dump variables + CNF fully
    with open(f"{out_prefix}_variables.txt", "w", encoding="utf-8") as f:
        f.write(f"TOTAL VARIABLES = {len(variables)}\n\n")
        for v in variables:
            f.write(repr(v) + "\n")

    with open(f"{out_prefix}_cnf.txt", "w", encoding="utf-8") as f:
        f.write(f"TOTAL CLAUSES = {len(cnf)}\n\n")
        for i, cl in enumerate(cnf):
            f.write(f"CLAUSE {i} (len={len(cl)}): {repr(cl)}\n")

    # 2) solve SAT
    sat, asg = solve_SAT(variables, cnf, {})

    with open(f"{out_prefix}_sat_result.txt", "w", encoding="utf-8") as f:
        f.write(f"SAT = {sat}\n")
        if asg is None:
            f.write("ASSIGNMENT = None\n")
        else:
            f.write(f"ASSIGNMENT SIZE = {len(asg)}\n\n")
            for k, v in asg.items():
                f.write(f"{repr(k)} = {v}\n")

    # 3) board
    if sat and asg is not None:
        board = numbers_assignment(variables, asg, p)
    else:
        board = None

    with open(f"{out_prefix}_board.txt", "w", encoding="utf-8") as f:
        f.write(f"BOARD = {board}\n")

    print("Saved full outputs to:")
    print(f"  {out_prefix}_variables.txt")
    print(f"  {out_prefix}_cnf.txt")
    print(f"  {out_prefix}_sat_result.txt")
    print(f"  {out_prefix}_board.txt")
