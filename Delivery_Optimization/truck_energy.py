import os
import sys
import numpy as np
import networkx as nx

import config as C


# ----------------------------------------------------------------------
# MOVESTAR access (import the function; bypass its file I/O)
# ----------------------------------------------------------------------
def _load_movestar():
    if C.MOVESTAR_DIR not in sys.path:
        sys.path.insert(0, C.MOVESTAR_DIR)
    import movestar as mv
    return mv


def _class_rate_J_per_m(mv, speed_ms, profile_seconds=120):
    if speed_ms <= 0:
        return 0.0
    speed = np.full(int(profile_seconds), float(speed_ms))  # m/s, 1 Hz
    out = mv.movestar(C.TRUCK_VEH_TYPE, speed)

    # "Emission Rate" row: total over the trip; index 5 = Energy(KJ)
    energy_kJ = out["Emission Rate"][0][5]
    dist_m = float(np.sum(speed))          # 1 Hz => sum of speeds = meters
    if dist_m <= 0:
        return 0.0
    energy_J = energy_kJ * C.KJ_TO_J
    return energy_J / dist_m               # J/m


def build_class_rates():
    mv = _load_movestar()
    rates = {}
    for cls, mph in C.SPEED_MPH.items():
        ms = mph * C.MPH_TO_MS
        rates[cls] = _class_rate_J_per_m(mv, ms)
    # default class rate too (for unmapped non-excluded classes)
    rates["_DEFAULT_"] = _class_rate_J_per_m(mv, C.DEFAULT_SPEED_MPH * C.MPH_TO_MS)
    print("[truck_energy] per-class J/m:",
          {k: round(v, 3) for k, v in rates.items()})
    return rates


# ----------------------------------------------------------------------
# Attach per-segment energy to the graph
# ----------------------------------------------------------------------
def annotate_graph_energy(G, class_rates):
    for u, v, d in G.edges(data=True):
        cls = d.get("road_class")
        rate = class_rates.get(cls, class_rates["_DEFAULT_"])
        d["energy"] = d["length"] * rate     # J
    return G


# ----------------------------------------------------------------------
# c_ij and tau_ij between a set of terminal nodes (hub, stops, or cells)
# ----------------------------------------------------------------------
def pairwise_costs(G, terminal_nodes, return_paths=False):
    terminals = list(terminal_nodes)
    c = {}
    tau = {}
    paths = {}
    for i in terminals:
        elen, epath = nx.single_source_dijkstra(G, i, weight="energy")
        for j in terminals:
            if j == i:
                continue
            if j not in elen:
                c[(i, j)] = float("inf")
                tau[(i, j)] = float("inf")
                if return_paths:
                    paths[(i, j)] = None
                continue
            c[(i, j)] = elen[j]
            path = epath[j]
            t = sum(G[path[k]][path[k+1]]["time"] for k in range(len(path)-1))
            tau[(i, j)] = t
            if return_paths:
                paths[(i, j)] = path
    if return_paths:
        return c, tau, paths
    return c, tau
