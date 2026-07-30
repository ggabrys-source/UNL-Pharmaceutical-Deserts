import numpy as np
import pandas as pd
import geopandas as gpd
import networkx as nx
from shapely.geometry import Point, LineString
from shapely.strtree import STRtree
from shapely.ops import nearest_points

import config as C


# ----------------------------------------------------------------------
# Loading
# ----------------------------------------------------------------------
def _load_layer(path, name):
    gdf = gpd.read_file(path)
    if gdf.crs is None:
        raise ValueError(f"{name} has no CRS; expected NAD83.")
    gdf = gdf.to_crs(epsg=C.TARGET_EPSG)
    return gdf


def load_all():
    roads = _load_layer(C.ROADS_GPKG, "roads")
    cells = _load_layer(C.CELLS_GPKG, "cells")
    hub   = _load_layer(C.HUB_GPKG,   "hub")
    return roads, cells, hub


# ----------------------------------------------------------------------
# Speed assignment
# ----------------------------------------------------------------------
def _segment_speed_ms(road_class):
    if road_class in C.EXCLUDE_CLASSES:
        return None  # excluded
    mph = C.SPEED_MPH.get(road_class, C.DEFAULT_SPEED_MPH)
    return mph * C.MPH_TO_MS


# ----------------------------------------------------------------------
# Node identity: snap a coordinate to an integer id
# ----------------------------------------------------------------------
class NodeIndex:
    def __init__(self, tol=C.NODE_SNAP_TOL):
        self.tol = tol
        self._coord_to_id = {}
        self.coords = []          # id -> (x, y)

    def _key(self, x, y):
        # round to tolerance grid so near-coincident endpoints collapse
        return (round(x / self.tol), round(y / self.tol))

    def get(self, x, y):
        k = self._key(x, y)
        if k not in self._coord_to_id:
            nid = len(self.coords)
            self._coord_to_id[k] = nid
            self.coords.append((x, y))
        return self._coord_to_id[k]

    def nearest(self, x, y):
        arr = np.asarray(self.coords)
        d = np.hypot(arr[:, 0] - x, arr[:, 1] - y)
        i = int(np.argmin(d))
        return i, float(d[i])


# ----------------------------------------------------------------------
# Graph construction
# ----------------------------------------------------------------------
def build_graph(roads):
    G = nx.Graph()
    nodeidx = NodeIndex()

    n_excluded = 0
    n_edges = 0
    for _, row in roads.iterrows():
        rc = row.get(C.COL_ROAD_CLASS)
        speed = _segment_speed_ms(rc)
        if speed is None:
            n_excluded += 1
            continue

        geom = row.geometry
        if geom is None or geom.is_empty:
            continue

        # handle MultiLineString by iterating parts
        parts = geom.geoms if geom.geom_type == "MultiLineString" else [geom]
        for part in parts:
            coords = list(part.coords)
            for a, b in zip(coords[:-1], coords[1:]):
                ua = nodeidx.get(a[0], a[1])
                ub = nodeidx.get(b[0], b[1])
                if ua == ub:
                    continue
                seglen = float(np.hypot(b[0] - a[0], b[1] - a[1]))
                segtime = seglen / speed
                # keep the cheaper edge if a duplicate appears
                if G.has_edge(ua, ub):
                    if seglen < G[ua][ub]["length"]:
                        G[ua][ub].update(length=seglen, speed=speed,
                                         time=segtime, road_class=rc)
                else:
                    G.add_edge(ua, ub, length=seglen, speed=speed,
                               time=segtime, road_class=rc)
                    n_edges += 1

    print(f"[graph] edges={n_edges}  nodes={G.number_of_nodes()}  "
          f"excluded_private_features={n_excluded}")
    return G, nodeidx


# ----------------------------------------------------------------------
# Connectivity
# ----------------------------------------------------------------------
def connectivity_report(G):
    comps = list(nx.connected_components(G))
    comps.sort(key=len, reverse=True)
    sizes = [len(c) for c in comps]
    print(f"[connectivity] components={len(comps)}  "
          f"largest={sizes[0] if sizes else 0}  "
          f"top5={sizes[:5]}")
    # node -> component index
    node_comp = {}
    for ci, comp in enumerate(comps):
        for n in comp:
            node_comp[n] = ci
    return comps, node_comp


# ----------------------------------------------------------------------
# Edge index over the MAIN component, for nearest-point-on-edge snapping
# ----------------------------------------------------------------------
def build_edge_index(G, nodeidx, main_component):
    node_xy = nodeidx.coords
    edge_lines = []
    edge_pairs = []
    for u, v, data in G.edges(data=True):
        if u in main_component and v in main_component:
            line = LineString([node_xy[u], node_xy[v]])
            edge_lines.append(line)
            edge_pairs.append((u, v))
    tree = STRtree(edge_lines)
    return tree, edge_lines, edge_pairs


def snap_points_to_edges(gdf, G, nodeidx, id_col, tree, edge_lines, edge_pairs,
                         label):
    rows = []
    for _, r in gdf.iterrows():
        g = r.geometry
        if g.geom_type != "Point":
            g = g.centroid
        p = Point(g.x, g.y)

        # nearest edge via STRtree; shapely 2.x returns an int index,
        # shapely 1.x returns the geometry itself
        res = tree.nearest(p)
        if isinstance(res, (int, np.integer)):
            idx = int(res)
        else:
            # geometry returned: find its index
            idx = edge_lines.index(res)
        line = edge_lines[idx]
        u, v = edge_pairs[idx]

        # projected point on that edge
        proj = line.interpolate(line.project(p))
        dist = p.distance(proj)

        # decide: snap to an existing endpoint if projection is essentially there,
        # else split the edge and insert a new node
        du = Point(nodeidx.coords[u]).distance(proj)
        dv = Point(nodeidx.coords[v]).distance(proj)
        if du < C.NODE_SNAP_TOL:
            node_id = u
        elif dv < C.NODE_SNAP_TOL:
            node_id = v
        else:
            node_id = nodeidx.get(proj.x, proj.y)
            if node_id != u and node_id != v:
                # split edge (u,v) at node_id
                data = G[u][v]
                speed = data["speed"]
                rc = data.get("road_class")
                d_un = Point(nodeidx.coords[u]).distance(proj)
                d_nv = Point(nodeidx.coords[v]).distance(proj)
                G.add_edge(u, node_id, length=d_un, speed=speed,
                           time=d_un/speed, road_class=rc)
                G.add_edge(node_id, v, length=d_nv, speed=speed,
                           time=d_nv/speed, road_class=rc)
                # keep original edge too (harmless; shortest path still valid)

        rows.append({
            "id": r[id_col] if id_col in gdf.columns else _,
            "x": g.x, "y": g.y,
            "node_id": node_id,
            "snap_dist_m": dist,
        })

    df = pd.DataFrame(rows)
    far = df[df.snap_dist_m > 805]  # 0.5 mi
    if len(far):
        print(f"[snap:{label}] NOTE {len(far)} points snapped >0.5 mi "
              f"(median {df.snap_dist_m.median():.0f} m, max {df.snap_dist_m.max():.0f} m)")
    else:
        print(f"[snap:{label}] all within 0.5 mi "
              f"(median {df.snap_dist_m.median():.0f} m)")
    return df


# ----------------------------------------------------------------------
# Convenience: full build
# ----------------------------------------------------------------------
def prepare():
    roads, cells, hub = load_all()
    G, nodeidx = build_graph(roads)
    comps, node_comp = connectivity_report(G)

    main_component = comps[0]  # largest
    tree, edge_lines, edge_pairs = build_edge_index(G, nodeidx, main_component)

    hub_id_col = C.COL_CELL_ID if C.COL_CELL_ID in hub.columns else hub.columns[0]
    cells_snap = snap_points_to_edges(cells, G, nodeidx, C.COL_CELL_ID,
                                      tree, edge_lines, edge_pairs, "cells")
    hub_snap   = snap_points_to_edges(hub, G, nodeidx, hub_id_col,
                                      tree, edge_lines, edge_pairs, "hub")

    # attach population back onto the cell snap table for later sampling
    if C.COL_POP in cells.columns:
        cells_snap[C.COL_POP] = cells[C.COL_POP].values
    if C.COL_HOUSING in cells.columns:
        cells_snap[C.COL_HOUSING] = cells[C.COL_HOUSING].values

    return {
        "roads": roads, "cells": cells, "hub": hub,
        "G": G, "nodeidx": nodeidx,
        "components": comps, "node_comp": node_comp,
        "cells_snap": cells_snap, "hub_snap": hub_snap,
    }


if __name__ == "__main__":
    prepare()
