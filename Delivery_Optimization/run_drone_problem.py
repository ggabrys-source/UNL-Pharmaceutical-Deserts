import os
import json
import csv
import numpy as np

import build_network
import siting
import drone_routing as DR
import truck_energy as TE
import truck_routing as TR
import config as C


class NpEncoder(json.JSONEncoder):
    def default(self, o):
        if isinstance(o, np.integer):
            return int(o)
        if isinstance(o, np.floating):
            return float(o)
        if isinstance(o, np.ndarray):
            return o.tolist()
        return super().default(o)


def export_routes_gpkg(all_case_geoms, path):
    import geopandas as gpd
    from shapely.geometry import LineString
    if not all_case_geoms:
        return
    gdf = gpd.GeoDataFrame(all_case_geoms, geometry="geometry",
                           crs=f"EPSG:{C.TARGET_EPSG}")
    gdf.to_file(path, driver="GPKG")
    print(f"[export] wrote {len(gdf)} drone legs -> {path}")


def main():
    # 1. network
    prep = build_network.prepare()

    # 2. siting (once)
    stops, cover, cell_xy = siting.run_siting(prep)

    # 2b. truck energy: per-class J/m via MOVESTAR, annotate graph edges once
    G = prep["G"]
    class_rates = TE.build_class_rates()
    TE.annotate_graph_energy(G, class_rates)
    hub_node = int(prep["hub_snap"]["node_id"].iloc[0])

    # map cell id -> index into cell_xy, for demand translation
    cells_snap = prep["cells_snap"]
    id_col = "id" if "id" in cells_snap.columns else C.COL_CELL_ID
    id_to_idx = {cid: i for i, cid in enumerate(cells_snap[id_col].to_numpy())}

    # 3. demand cases
    cases = siting.sample_demand(cells_snap)

    # persist siting + demand
    with open(os.path.join(C.OUT_DIR, "siting.json"), "w") as f:
        json.dump({"stops": [int(s) for s in stops],
                   "n_stops": len(stops),
                   "R_miles": C.R_COVER_MILES}, f, indent=2, cls=NpEncoder)

    # persist node_id -> (x, y) so any node_id in results is resolvable later
    import csv as _csv
    with open(os.path.join(C.OUT_DIR, "node_coords.csv"), "w", newline="") as f:
        w = _csv.writer(f)
        w.writerow(["node_id", "x", "y"])
        for nid, (x, y) in enumerate(prep["nodeidx"].coords):
            w.writerow([nid, x, y])

    # reverse map: cell index -> cell id, for writing ids into routes
    idx_to_id = {i: cid for cid, i in id_to_idx.items()}
    # cell index -> snapped network node id (for Config 2 truck-to-cell routing)
    cell_node = {i: int(nid) for i, nid in
                 enumerate(prep["cells_snap"]["node_id"].to_numpy())}

    node_xy = prep["nodeidx"].coords
    all_case_geoms = []
    summary_rows = []
    for n in C.DEMAND_SIZES:
        demand_ids = cases[n]
        demand_idxs = [id_to_idx[c] for c in demand_ids]

        # per-case: minimal subset of sited stops covering this demand
        active = siting.solve_active_stops(demand_idxs, stops, cover)

        # assign to nearest ACTIVE stop
        Q = DR.assign_cells_to_stops(demand_idxs, active, cover, cell_xy,
                                     prep["nodeidx"])

        # solve per-stop VRPs
        result = DR.run_drone_routing(Q, cell_xy, prep["nodeidx"])

        # --- build leg geometries + translate cell indices -> cell ids ---
        from shapely.geometry import LineString
        for stop_node, v in result["per_stop"].items():
            sx, sy = node_xy[stop_node]
            for route in v["routes"]:
                # route["cells"] is a list of cell INDICES in visit order
                seq_idx = route["cells"]
                seq_ids = [int(idx_to_id[ci]) for ci in seq_idx]
                route["cells"] = seq_ids  # overwrite with ids for output
                # geometry: stop -> c1 -> c2 -> ... -> stop
                pts = [(sx, sy)]
                for ci in seq_idx:
                    pts.append((cell_xy[ci, 0], cell_xy[ci, 1]))
                pts.append((sx, sy))
                for leg_i in range(len(pts) - 1):
                    all_case_geoms.append({
                        "demand_size": n,
                        "stop_node": int(stop_node),
                        "drone": route["drone"],
                        "leg": leg_i,
                        "kind": "drone",
                        "geometry": LineString([pts[leg_i], pts[leg_i + 1]]),
                    })

        # aggregate drone time: sum over stops of the stop's slowest route
        drone_time_component = sum(v["time"] for v in result["per_stop"].values())
        n_active_stops = sum(1 for v in result["per_stop"].values() if v["n_drones"] > 0)
        unserved = [c for v in result["per_stop"].values()
                    for c in v.get("unserved", [])]

        # ---- Level 2: truck tours hub + ACTIVE stops (drone config) ----
        active_used = [s for s, v in result["per_stop"].items() if v["n_drones"] > 0]
        term_l2 = [hub_node] + active_used
        c_l2, tau_l2, paths_l2 = TE.pairwise_costs(G, term_l2, return_paths=True)
        truck_l2 = TR.solve_tsp(active_used, hub_node, c_l2, tau_l2)

        # combined drone-config metrics
        combo_energy = truck_l2["energy_J"] + result["drone_energy"]
        combo_time = truck_l2["time_s"] + drone_time_component

        # ---- Config 2: truck-only, tours hub + demand cells ----
        demand_nodes = list({cell_node[i] for i in demand_idxs})
        term_c2 = [hub_node] + demand_nodes
        c_c2, tau_c2, paths_c2 = TE.pairwise_costs(G, term_c2, return_paths=True)
        truck_only = TR.solve_tsp(demand_nodes, hub_node, c_c2, tau_c2)
        truckonly_energy = truck_only["energy_J"]
        truckonly_time = truck_only["time_s"] + len(demand_idxs) * C.T_STOP_SEC

        # --- truck tour geometries (road-following via Dijkstra paths) ---
        from shapely.geometry import LineString as _LS
        def _tour_legs(order, kind, paths):
            for k in range(len(order)):
                a = order[k]
                b = order[(k + 1) % len(order)]  # close loop to hub
                p = paths.get((a, b))
                if p and len(p) >= 2:
                    coords = [node_xy[nd] for nd in p]   # real road path
                else:
                    coords = [node_xy[a], node_xy[b]]    # fallback straight
                all_case_geoms.append({
                    "demand_size": n, "stop_node": -1, "drone": -1,
                    "leg": k, "kind": kind,
                    "geometry": _LS(coords),
                })
        _tour_legs(truck_l2["order"], "truck_l2", paths_l2)
        _tour_legs(truck_only["order"], "truck_only", paths_c2)

        case_out = {
            "demand_size": n,
            "drone_energy_J": result["drone_energy"],
            "truck_l2_energy_J": truck_l2["energy_J"],
            "combined_energy_J": combo_energy,
            "combined_time_s": combo_time,
            "truck_only_energy_J": truckonly_energy,
            "truck_only_time_s": truckonly_time,
            "total_drones": result["drone_total_drones"],
            "active_stops": n_active_stops,
            "drone_time_component_s": drone_time_component,
            "truck_l2_order": [int(x) for x in truck_l2["order"]],
            "truck_only_order": [int(x) for x in truck_only["order"]],
            "truck_l2_gap": truck_l2.get("gap"),
            "truck_l2_status": truck_l2.get("status"),
            "truck_only_gap": truck_only.get("gap"),
            "truck_only_status": truck_only.get("status"),
            "unserved_cells": unserved,
            "per_stop": {int(s): {"n_drones": v["n_drones"],
                                  "energy_J": v["energy"],
                                  "time_s": v["time"],
                                  "routes": v["routes"]}
                         for s, v in result["per_stop"].items()},
        }
        with open(os.path.join(C.OUT_DIR, f"drone_case_{n}.json"), "w") as f:
            json.dump(case_out, f, indent=2, cls=NpEncoder)

        energy_saved_pct = (truckonly_energy - combo_energy) / truckonly_energy * 100 \
            if truckonly_energy else 0.0
        time_penalty_pct = (combo_time - truckonly_time) / truckonly_time * 100 \
            if truckonly_time else 0.0

        summary_rows.append({
            "demand_size": n,
            "active_stops_chosen": len(active),
            "drone_energy_MJ": round(result["drone_energy"]/1e6, 4),
            "truck_l2_energy_MJ": round(truck_l2["energy_J"]/1e6, 4),
            "combined_energy_MJ": round(combo_energy/1e6, 4),
            "combined_time_min": round(combo_time/60, 2),
            "truck_only_energy_MJ": round(truckonly_energy/1e6, 4),
            "truck_only_time_min": round(truckonly_time/60, 2),
            "energy_saved_pct": round(energy_saved_pct, 2),
            "time_penalty_pct": round(time_penalty_pct, 2),
            "truck_l2_gap_pct": round(truck_l2["gap"]*100, 2) if truck_l2.get("gap") is not None else None,
            "truck_only_gap_pct": round(truck_only["gap"]*100, 2) if truck_only.get("gap") is not None else None,
            "total_drones": result["drone_total_drones"],
            "n_unserved": len(unserved),
        })
        def _g(x): return "opt" if (x.get("gap") is not None and x["gap"] <= C.MIP_GAP) else \
                          (f"gap={x['gap']*100:.1f}%" if x.get("gap") is not None else x.get("status","?"))
        print(f"[case {n}] combined={combo_energy/1e6:.2f}MJ "
              f"(truck {truck_l2['energy_J']/1e6:.2f}+drone {result['drone_energy']/1e6:.2f}) "
              f"| truck-only={truckonly_energy/1e6:.2f}MJ  "
              f"drones={result['drone_total_drones']}  unserved={len(unserved)}  "
              f"[L2:{_g(truck_l2)} C2:{_g(truck_only)}]")

    # CSV summary
    with open(os.path.join(C.OUT_DIR, "summary_drone.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(summary_rows[0].keys()))
        w.writeheader()
        w.writerows(summary_rows)

    # drone legs as a GeoPackage for QGIS
    export_routes_gpkg(all_case_geoms,
                       os.path.join(C.OUT_DIR, "drone_routes.gpkg"))

    print(f"[done] wrote results to {C.OUT_DIR}")


if __name__ == "__main__":
    main()
