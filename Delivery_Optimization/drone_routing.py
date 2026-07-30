import numpy as np
import pyomo.environ as pyo

import config as C
from solver import make_solver


# ----------------------------------------------------------------------
# Assign each demand cell to nearest covering stop  ->  Q_i
# ----------------------------------------------------------------------
def assign_cells_to_stops(demand_cell_idxs, stops, cover, cell_xy, nodeidx):
    node_xy = np.asarray(nodeidx.coords)
    stop_set = set(stops)
    Q = {s: [] for s in stops}

    for ci in demand_cell_idxs:
        covering = [s for s in cover[ci] if s in stop_set]
        if not covering:
            # should not happen: siting covered all cells
            raise RuntimeError(f"cell {ci} not covered by any opened stop")
        cx, cy = cell_xy[ci]
        # nearest covering stop by Euclidean distance
        best = min(covering,
                   key=lambda s: np.hypot(node_xy[s, 0] - cx, node_xy[s, 1] - cy))
        Q[best].append(ci)
    return Q


# ----------------------------------------------------------------------
# Per-stop VRP
# ----------------------------------------------------------------------
def _dist_matrix(stop_node, cell_idxs, cell_xy, nodeidx):
    node_xy = np.asarray(nodeidx.coords)
    pts = [ (node_xy[stop_node,0], node_xy[stop_node,1]) ]
    for ci in cell_idxs:
        pts.append((cell_xy[ci,0], cell_xy[ci,1]))
    pts = np.asarray(pts)
    n = len(pts)
    D = np.zeros((n, n))
    for a in range(n):
        D[a] = np.hypot(pts[:,0]-pts[a,0], pts[:,1]-pts[a,1])
    return D


def solve_stop_vrp(stop_node, cell_idxs, cell_xy, nodeidx, max_drones=None):
    if len(cell_idxs) == 0:
        return {"routes": [], "energy": 0.0, "n_drones": 0, "time": 0.0}

    D = _dist_matrix(stop_node, cell_idxs, cell_xy, nodeidx)  # meters
    n = D.shape[0]                # node 0 = stop, 1..n-1 = cells
    cells = list(range(1, n))
    e  = C.E_FLIGHT
    B  = C.B_BATTERY
    Esq = C.E_SQ

    # --- Filter: drop cells unreachable even as a lone out-and-back ---
    infeasible = [c for c in cells if 2*e*D[0, c] + Esq > B]
    if infeasible:
        # These cells can't be served from this stop at all -> siting/R mismatch.
        # Report and drop; they will show as unserved.
        print(f"[vrp:stop {stop_node}] WARNING {len(infeasible)} cells exceed "
              f"battery even alone; unserved: {[cell_idxs[c-1] for c in infeasible]}")
    serv = [c for c in cells if c not in infeasible]
    if not serv:
        return {"routes": [], "energy": 0.0, "n_drones": 0, "time": 0.0,
                "unserved": [cell_idxs[c-1] for c in infeasible]}

    # upper bound on drones: worst case one per served cell
    K = max_drones or len(serv)

    m = pyo.ConcreteModel()
    m.N = pyo.Set(initialize=[0] + serv)          # stop + serviceable cells
    m.C = pyo.Set(initialize=serv)                # demand cells only
    m.R = pyo.RangeSet(0, K-1)                     # drone/route pool

    # arc set: allow (a,b) a!=b within N. Filter 2: forbid legs that can't
    # return to stop within budget even minimally (stop->a->b->stop).
    def arc_ok(a, b):
        if a == b:
            return False
        return (e*(D[0,a] + D[a,b] + D[b,0]) + Esq*(1 if a in serv else 0)
                + (Esq if b in serv else 0)) <= B
    arcs = [(a,b) for a in m.N for b in m.N if arc_ok(a,b)]
    m.A = pyo.Set(initialize=arcs, dimen=2)

    # z[r,a,b] = 1 if drone r flies arc a->b
    m.z = pyo.Var(m.R, m.A, domain=pyo.Binary)
    # MTZ position of a cell on its route (per stop, shared bound)
    m.u = pyo.Var(m.C, domain=pyo.NonNegativeReals, bounds=(1, len(serv)))

    # ---- objective: total flight + delivery energy over all drones ----
    def obj_rule(mm):
        flight = sum(e*D[a,b]*mm.z[r,a,b] for r in mm.R for (a,b) in mm.A)
        deliver = sum(Esq*mm.z[r,a,b] for r in mm.R for (a,b) in mm.A if b in serv)
        return flight + deliver
    m.obj = pyo.Objective(rule=obj_rule, sense=pyo.minimize)

    # ---- each demand cell entered exactly once (across all drones) ----
    def visit_rule(mm, c):
        return sum(mm.z[r,a,c] for r in mm.R for a in mm.N if (a,c) in mm.A) == 1
    m.visit = pyo.Constraint(m.C, rule=visit_rule)

    # ---- flow conservation per drone at every node ----
    def flow_rule(mm, r, h):
        ins  = sum(mm.z[r,a,h] for a in mm.N if (a,h) in mm.A)
        outs = sum(mm.z[r,h,b] for b in mm.N if (h,b) in mm.A)
        return ins == outs
    m.flow = pyo.Constraint(m.R, m.N, rule=flow_rule)

    # ---- each drone leaves the stop at most once (one route per drone) ----
    def depart_rule(mm, r):
        return sum(mm.z[r,0,b] for b in mm.N if (0,b) in mm.A) <= 1
    m.depart = pyo.Constraint(m.R, rule=depart_rule)

    # ---- battery limit per drone ----
    def batt_rule(mm, r):
        flight = sum(e*D[a,b]*mm.z[r,a,b] for (a,b) in mm.A)
        deliver = sum(Esq*mm.z[r,a,b] for (a,b) in mm.A if b in serv)
        return flight + deliver <= B
    m.batt = pyo.Constraint(m.R, rule=batt_rule)

    # ---- MTZ subtour elimination (shared u; anchored at stop 0) ----
    # applies across cells regardless of drone: a cell's position increases
    # along whichever route uses the arc.
    nC = len(serv)
    def mtz_rule(mm, a, b):
        if a in serv and b in serv and a != b and (a, b) in mm.A:
            return mm.u[a] - mm.u[b] + nC*sum(mm.z[r,a,b] for r in mm.R) <= nC - 1
        return pyo.Constraint.Skip
    m.mtz = pyo.Constraint(m.C, m.C, rule=mtz_rule)

    # ---- symmetry break: drone r used only if drone r-1 used ----
    def sym_rule(mm, r):
        if r == 0:
            return pyo.Constraint.Skip
        this = sum(mm.z[r,0,b] for b in mm.N if (0,b) in mm.A)
        prev = sum(mm.z[r-1,0,b] for b in mm.N if (0,b) in mm.A)
        return this <= prev
    m.sym = pyo.Constraint(m.R, rule=sym_rule)

    solver = make_solver()
    res = solver.solve(m, tee=False)

    # ---- extract routes ----
    routes = []
    total_energy = 0.0
    n_used = 0
    route_times = []
    for r in m.R:
        succ = {}
        used = False
        for (a,b) in m.A:
            if pyo.value(m.z[r,a,b]) > 0.5:
                succ[a] = b
                used = True
        if not used:
            continue
        n_used += 1
        # walk from stop 0
        seq = []
        cur = 0
        steps = 0
        route_len = 0.0
        route_deliv = 0
        while True:
            nxt = succ.get(cur, None)
            if nxt is None:
                break
            route_len += D[cur, nxt]
            if nxt in serv:
                seq.append(cell_idxs[nxt-1])  # back to original cell index
                route_deliv += 1
            cur = nxt
            steps += 1
            if cur == 0 or steps > len(serv)+2:
                break
        energy = e*route_len + Esq*route_deliv
        total_energy += energy
        rtime = route_len / C.V_DRONE + C.T_HOVER*route_deliv
        route_times.append(rtime)
        routes.append({"drone": r, "cells": seq, "energy": energy,
                       "flight_m": route_len, "deliveries": route_deliv,
                       "time_s": rtime})

    # per-stop time: serial loading (n_drones * t_init) + slowest flight (parallel)
    flight_component = max(route_times) if route_times else 0.0
    stop_time = n_used * C.T_INIT_SEC + flight_component
    return {"routes": routes, "energy": total_energy, "n_drones": n_used,
            "time": stop_time,
            "unserved": [cell_idxs[c-1] for c in infeasible] if infeasible else []}


def run_drone_routing(Q, cell_xy, nodeidx):
    per_stop = {}
    total_energy = 0.0
    total_drones = 0
    for stop_node, cell_idxs in Q.items():
        r = solve_stop_vrp(stop_node, cell_idxs, cell_xy, nodeidx)
        per_stop[stop_node] = r
        total_energy += r["energy"]
        total_drones += r["n_drones"]
    return {"per_stop": per_stop,
            "drone_energy": total_energy,
            "drone_total_drones": total_drones}
