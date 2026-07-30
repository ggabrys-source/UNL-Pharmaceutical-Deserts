import os
import re
import json
import glob
import csv
import numpy as np
import geopandas as gpd
import networkx as nx
from shapely.geometry import LineString

import config as C
import build_network as BN
import network_heal as NH
import truck_energy as TE


# ----------------------------------------------------------------------
# Economic constants (running-cost comparison; no capital)
# ----------------------------------------------------------------------
GAS_PRICE   = 4.00       # $/gallon
MJ_PER_GAL  = 121.0      # gasoline lower heating value (MJ/gal)
DRIVER_WAGE = 25.0       # $/hour
SERVICE_BASE = 40.00     # $ base fare per round trip (documented NE medical-transport rate)
SERVICE_PERMILE = 1.00   # $ per mile
MILE_M = 1609.34

# wider road network for cell<->pharmacy routing
ROADS_PHARM_GPKG = os.path.join(C.GPKG_DIR, "road_centerlines_pharm.gpkg")
PHARM_GPKG       = os.path.join(C.GPKG_DIR, "pharmacies_pd.gpkg")


def energy_to_cost(energy_J):
    gal = (energy_J / 1e6) / MJ_PER_GAL
    return gal * GAS_PRICE


# ----------------------------------------------------------------------
# Build the wider road graph and snap pharmacies + cells onto it
# ----------------------------------------------------------------------
def build_pharmacy_graph():
    roads = gpd.read_file(ROADS_PHARM_GPKG).to_crs(epsg=C.TARGET_EPSG)
    pharm = gpd.read_file(PHARM_GPKG).to_crs(epsg=C.TARGET_EPSG)
    cells = gpd.read_file(C.CELLS_GPKG).to_crs(epsg=C.TARGET_EPSG)

    G, nodeidx = BN.build_graph(roads)          # excludes PRIVATE, builds edges
    NH.heal_network(G, nodeidx, heal_tol=25.0)  # bridge NG911 gaps so pharmacies snap correctly
    comps, node_comp = BN.connectivity_report(G)
    main = comps[0]
    tree, edge_lines, edge_pairs = BN.build_edge_index(G, nodeidx, main)

    # MOVESTAR energy weights on this graph too
    class_rates = TE.build_class_rates()
    TE.annotate_graph_energy(G, class_rates)

    # snap pharmacies and cells to the wider network
    ph_snap = BN.snap_points_to_edges(pharm, G, nodeidx,
                                      "osm_id" if "osm_id" in pharm.columns else pharm.columns[0],
                                      tree, edge_lines, edge_pairs, "pharmacies")
    cell_snap = BN.snap_points_to_edges(cells, G, nodeidx, C.COL_CELL_ID,
                                        tree, edge_lines, edge_pairs, "cells_pharm")
    return G, nodeidx, ph_snap, cell_snap, cells


# ----------------------------------------------------------------------
# Nearest-pharmacy round-trip energy and distance per cell
# ----------------------------------------------------------------------
def cell_to_nearest_pharmacy(G, cell_node, pharm_nodes):
    # single-source over energy, then pick nearest pharmacy by energy
    elen, epath = nx.single_source_dijkstra(G, cell_node, weight="energy")
    best = None
    for pn in pharm_nodes:
        if pn in elen:
            if best is None or elen[pn] < best[0]:
                path = epath[pn]
                dist = sum(G[path[k]][path[k+1]]["length"] for k in range(len(path)-1))
                best = (elen[pn], dist)
    if best is None:
        return None, None
    e_one, d_one = best
    return 2.0 * e_one, 2.0 * d_one     # round trip


def build_selfdrive_table(G, nodeidx, ph_snap, cell_snap):
    pharm_nodes = list(ph_snap["node_id"].astype(int).unique())
    out = {}
    cell_ids = cell_snap[C.COL_CELL_ID].to_numpy()
    cell_nodes = cell_snap["node_id"].astype(int).to_numpy()
    for i, (cid, cnode) in enumerate(zip(cell_ids, cell_nodes)):
        e, d = cell_to_nearest_pharmacy(G, int(cnode), pharm_nodes)
        out[cid] = (e, d)
    return out


# ----------------------------------------------------------------------
# Pull system energy/time per scenario from the main results JSONs
# ----------------------------------------------------------------------
def load_system_results():
    rows = {}
    for f in glob.glob(os.path.join(C.OUT_DIR, "drone_case_*.json")):
        d = json.load(open(f))
        n = d["demand_size"]
        rows[n] = {
            "combined_energy_J": d.get("combined_energy_J"),
            "combined_time_s":   d.get("combined_time_s"),
            "truck_only_energy_J": d.get("truck_only_energy_J"),
            "truck_only_time_s":   d.get("truck_only_time_s"),
            "demand_ids": None,   # filled below if present
        }
    return rows


def demand_ids_for_scenario(n):
    f = os.path.join(C.OUT_DIR, f"drone_case_{n}.json")
    d = json.load(open(f))
    ids = set()
    for s, v in d.get("per_stop", {}).items():
        for r in v.get("routes", []):
            ids.update(r.get("cells", []))
    return ids


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------
def main():
    G, nodeidx, ph_snap, cell_snap, cells = build_pharmacy_graph()
    selfdrive = build_selfdrive_table(G, nodeidx, ph_snap, cell_snap)
    systems = load_system_results()

    out_rows = []
    for n in sorted(systems.keys()):
        s = systems[n]
        D = n  # per-person denominator = number of deliveries simulated

        # ---- delivery systems, per person ----
        to_fuel = energy_to_cost(s["truck_only_energy_J"])
        to_labor = (s["truck_only_time_s"] / 3600.0) * DRIVER_WAGE
        to_cost_pp = (to_fuel + to_labor) / D
        to_energy_pp = s["truck_only_energy_J"] / D

        td_fuel = energy_to_cost(s["combined_energy_J"])
        td_labor = (s["combined_time_s"] / 3600.0) * DRIVER_WAGE
        td_cost_pp = (td_fuel + td_labor) / D
        td_energy_pp = s["combined_energy_J"] / D

        # ---- self-drive status quo, per person (mean over the scenario's cells) ----
        dids = demand_ids_for_scenario(n)
        sd_energies = []
        sd_costs = []
        svc_costs = []
        for cid in dids:
            e, dm = selfdrive.get(cid, (None, None))
            if e is None:
                continue
            sd_energies.append(e)
            sd_costs.append(energy_to_cost(e))
            miles = dm / MILE_M
            # service: $40 base each way (x2) + $1/mile on round-trip miles
            svc_costs.append(SERVICE_BASE + SERVICE_PERMILE * miles)
        sd_energy_pp = float(np.mean(sd_energies)) if sd_energies else float("nan")
        sd_cost_pp   = float(np.mean(sd_costs)) if sd_costs else float("nan")
        svc_cost_pp  = float(np.mean(svc_costs)) if svc_costs else float("nan")

        out_rows.append({
            "demand_size": n,
            # energy per person (MJ)
            "truck_only_energy_pp_MJ": round(to_energy_pp/1e6, 4),
            "truck_drone_energy_pp_MJ": round(td_energy_pp/1e6, 4),
            "self_drive_energy_pp_MJ": round(sd_energy_pp/1e6, 4),
            # cost per person ($)
            "truck_only_cost_pp": round(to_cost_pp, 2),
            "truck_drone_cost_pp": round(td_cost_pp, 2),
            "self_drive_cost_pp": round(sd_cost_pp, 2),
            "service_cost_pp": round(svc_cost_pp, 2),
        })
        print(f"[|D|={n}] energy/person MJ: truck={to_energy_pp/1e6:.1f} "
              f"drone={td_energy_pp/1e6:.1f} self={sd_energy_pp/1e6:.1f} | "
              f"cost/person $: truck={to_cost_pp:.2f} drone={td_cost_pp:.2f} "
              f"self={sd_cost_pp:.2f} service={svc_cost_pp:.2f}")

    out = os.path.join(C.OUT_DIR, "per_person_summary.csv")
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(out_rows[0].keys()))
        w.writeheader()
        w.writerows(out_rows)
    print(f"[done] wrote {out}")


if __name__ == "__main__":
    main()
