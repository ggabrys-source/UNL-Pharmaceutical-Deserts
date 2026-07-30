import pyomo.environ as pyo
import config as C
from solver import make_solver


def solve_tsp(nodes, hub, c, tau):
    V = [hub] + list(nodes)
    n = len(V)
    if n == 1:
        return {"order": [hub], "energy_J": 0.0, "time_s": 0.0,
                "status": "trivial", "gap": 0.0}
    if n == 2:
        # hub <-> single node, out and back
        a = V[1]
        e = c[(hub, a)] + c[(a, hub)]
        t = tau[(hub, a)] + tau[(a, hub)]
        return {"order": [hub, a], "energy_J": e, "time_s": t,
                "status": "trivial", "gap": 0.0}

    m = pyo.ConcreteModel()
    m.V = pyo.Set(initialize=V)
    m.A = pyo.Set(initialize=[(i, j) for i in V for j in V if i != j], dimen=2)

    m.x = pyo.Var(m.A, domain=pyo.Binary)
    # MTZ position vars for non-hub nodes
    m.p = pyo.Var(nodes, domain=pyo.NonNegativeReals, bounds=(1, n - 1))

    m.obj = pyo.Objective(
        expr=sum(c[(i, j)] * m.x[i, j] for (i, j) in m.A), sense=pyo.minimize)

    # each node entered exactly once
    def _in(mm, j):
        return sum(mm.x[i, j] for i in V if i != j) == 1
    m.deg_in = pyo.Constraint(m.V, rule=_in)

    # each node left exactly once
    def _out(mm, i):
        return sum(mm.x[i, j] for j in V if j != i) == 1
    m.deg_out = pyo.Constraint(m.V, rule=_out)

    # MTZ subtour elimination (nodes excluding hub)
    def _mtz(mm, i, j):
        if i != j:
            return mm.p[i] - mm.p[j] + (n - 1) * mm.x[i, j] <= n - 2
        return pyo.Constraint.Skip
    m.mtz = pyo.Constraint(nodes, nodes, rule=_mtz)

    solver = make_solver()
    results = solver.solve(m, tee=False)

    # capture status + optimality gap if available
    status = str(results.solver.termination_condition) \
        if hasattr(results, "solver") else "unknown"
    gap = None
    try:
        ub = results.problem.upper_bound
        lb = results.problem.lower_bound
        if ub is not None and lb is not None and abs(ub) > 1e-9:
            gap = abs(ub - lb) / abs(ub)
    except Exception:
        gap = None

    # reconstruct tour from hub
    succ = {}
    for (i, j) in m.A:
        if pyo.value(m.x[i, j]) > 0.5:
            succ[i] = j
    order = [hub]
    cur = hub
    for _ in range(n):
        nxt = succ.get(cur)
        if nxt is None or nxt == hub:
            break
        order.append(nxt)
        cur = nxt

    energy = sum(c[(order[k], order[k+1])] for k in range(len(order)-1))
    energy += c[(order[-1], hub)]      # close the loop
    time = sum(tau[(order[k], order[k+1])] for k in range(len(order)-1))
    time += tau[(order[-1], hub)]

    return {"order": order, "energy_J": energy, "time_s": time,
            "status": status, "gap": gap}
