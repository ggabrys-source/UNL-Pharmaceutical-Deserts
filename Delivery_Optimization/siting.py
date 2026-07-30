import numpy as np
import pandas as pd
import pyomo.environ as pyo

import config as C
from solver import make_solver


# ----------------------------------------------------------------------
# Demand sampling: probability proportional to POP20, without replacement,
# nested across sizes, fixed seed.
# ----------------------------------------------------------------------
def sample_demand(cells_snap):
    rng = np.random.default_rng(C.SEED)
    pop = cells_snap[C.COL_POP].to_numpy(dtype=float)
    ids = cells_snap[C.COL_CELL_ID].to_numpy() if C.COL_CELL_ID in cells_snap.columns \
          else cells_snap["id"].to_numpy()

    # log weighting: capped so max/min selection ratio ~= 4.5x
    weights = np.log(pop + C.LOG_OFFSET)
    w = weights / weights.sum()
    max_n = max(C.DEMAND_SIZES)

    if C.NESTED:
        # single weighted ordering without replacement; slice prefixes
        order = rng.choice(len(ids), size=max_n, replace=False, p=w)
        cases = {n: set(ids[order[:n]]) for n in C.DEMAND_SIZES}
    else:
        cases = {}
        for n in C.DEMAND_SIZES:
            pick = rng.choice(len(ids), size=n, replace=False, p=w)
            cases[n] = set(ids[pick])
    return cases


# ----------------------------------------------------------------------
# Candidate/coverage geometry
# ----------------------------------------------------------------------
def build_coverage(cells_snap, nodeidx, node_comp):
    cell_xy = cells_snap[["x", "y"]].to_numpy()
    node_xy = np.asarray(nodeidx.coords)

    # prune candidate nodes to those within R of at least one cell
    # (brute force is fine at this scale; vectorize per cell)
    R = C.R_COVER
    cand_mask = np.zeros(len(node_xy), dtype=bool)
    cover = {i: set() for i in range(len(cell_xy))}

    for ci, (cx, cy) in enumerate(cell_xy):
        d = np.hypot(node_xy[:, 0] - cx, node_xy[:, 1] - cy)
        within = np.where(d <= R)[0]
        cand_mask[within] = True
        cover[ci].update(int(j) for j in within)

    cand_nodes = [int(j) for j in np.where(cand_mask)[0]]
    # sanity: every cell must have at least one candidate, else infeasible
    empties = [ci for ci, s in cover.items() if not s]
    if empties:
        print(f"[siting] WARNING {len(empties)} cells have NO candidate within "
              f"{C.R_COVER_MILES} mi; set-cover will be infeasible for them")
    print(f"[siting] candidates={len(cand_nodes)}  cells={len(cell_xy)}")
    return cand_nodes, cover, cell_xy


# ----------------------------------------------------------------------
# Set-cover MIP: minimize number of opened stops s.t. every cell covered.
# ----------------------------------------------------------------------
def solve_siting(cand_nodes, cover):
    m = pyo.ConcreteModel()
    m.J = pyo.Set(initialize=cand_nodes)              # candidate stop nodes
    m.K = pyo.Set(initialize=list(cover.keys()))      # cells

    m.s = pyo.Var(m.J, domain=pyo.Binary)

    m.obj = pyo.Objective(expr=sum(m.s[j] for j in m.J), sense=pyo.minimize)

    def _cover_rule(mm, k):
        js = cover[k]
        if not js:
            return pyo.Constraint.Skip
        return sum(mm.s[j] for j in js) >= 1
    m.cover = pyo.Constraint(m.K, rule=_cover_rule)

    solver = make_solver()
    solver.solve(m, tee=False)

    opened = [j for j in cand_nodes if pyo.value(m.s[j]) > 0.5]
    print(f"[siting] opened stops = {len(opened)}")
    return opened


def solve_active_stops(demand_cell_idxs, stops, cover):
    stop_set = set(stops)
    # coverage restricted to sited stops
    dcover = {}
    for ci in demand_cell_idxs:
        opts = [s for s in cover[ci] if s in stop_set]
        if not opts:
            raise RuntimeError(f"demand cell {ci} not covered by any sited stop")
        dcover[ci] = opts

    m = pyo.ConcreteModel()
    m.J = pyo.Set(initialize=list(stops))
    m.K = pyo.Set(initialize=list(demand_cell_idxs))
    m.s = pyo.Var(m.J, domain=pyo.Binary)
    m.obj = pyo.Objective(expr=sum(m.s[j] for j in m.J), sense=pyo.minimize)

    def _cov(mm, k):
        return sum(mm.s[j] for j in dcover[k]) >= 1
    m.cover = pyo.Constraint(m.K, rule=_cov)

    solver = make_solver()
    solver.solve(m, tee=False)
    active = [j for j in stops if pyo.value(m.s[j]) > 0.5]
    return active


def run_siting(prep):
    cand_nodes, cover, cell_xy = build_coverage(
        prep["cells_snap"], prep["nodeidx"], prep["node_comp"])
    stops = solve_siting(cand_nodes, cover)
    return stops, cover, cell_xy


if __name__ == "__main__":
    import build_network
    prep = build_network.prepare()
    stops, cover, cell_xy = run_siting(prep)
    cases = sample_demand(prep["cells_snap"])
    for n, s in cases.items():
        print(f"[demand] |D|={n}: {len(s)} cells")
