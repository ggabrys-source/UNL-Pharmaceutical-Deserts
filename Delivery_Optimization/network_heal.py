import numpy as np
import networkx as nx
import config as C


def heal_network(G, nodeidx, heal_tol=25.0):
    from scipy.spatial import cKDTree

    comps = list(nx.connected_components(G))
    comps.sort(key=len, reverse=True)
    node_comp = {}
    for ci, comp in enumerate(comps):
        for n in comp:
            node_comp[n] = ci

    if len(comps) == 1:
        print("[heal] already connected")
        return G

    coords = np.asarray(nodeidx.coords)
    tree = cKDTree(coords)
    pairs = tree.query_pairs(r=heal_tol, output_type='ndarray')

    added = 0
    for a, b in pairs:
        if a not in node_comp or b not in node_comp:
            continue
        if node_comp[a] != node_comp[b]:
            d = float(np.hypot(coords[a,0]-coords[b,0], coords[a,1]-coords[b,1]))
            if not G.has_edge(a, b):
                speed = C.DEFAULT_SPEED_MPH * C.MPH_TO_MS
                G.add_edge(a, b, length=max(d, 0.1), speed=speed,
                           time=max(d, 0.1)/speed, road_class="LOCAL")
                added += 1
                ca, cb = node_comp[a], node_comp[b]
                for n in list(node_comp):
                    if node_comp[n] == cb:
                        node_comp[n] = ca
    print(f"[heal] added {added} connectors at tol={heal_tol} m")
    return G
