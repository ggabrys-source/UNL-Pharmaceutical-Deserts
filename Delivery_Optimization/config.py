# All tunable parameters and paths for the delivery optimization.
import os
import math

# Paths are relative to this file, so the repo runs anywhere.
HERE      = os.path.dirname(os.path.abspath(__file__))
GPKG_DIR  = os.path.join(HERE, "geopackages")
MOVESTAR_DIR = os.path.join(HERE, "MOVESTAR")

# Battery/radius scale factor for sensitivity runs (env var, default 1.0).
BATT_MULT = float(os.environ.get("BATT_MULT", "1.0"))
OUT_DIR   = os.path.join(HERE, "results" if BATT_MULT == 1.0
                         else f"results_B{BATT_MULT:g}")

ROADS_GPKG = os.path.join(GPKG_DIR, "road_centerlines.gpkg")
CELLS_GPKG = os.path.join(GPKG_DIR, "cell_centroids.gpkg")
HUB_GPKG   = os.path.join(GPKG_DIR, "main_hub.gpkg")

# Geopackage column names
COL_ROAD_CLASS = "ST_CLASS"
COL_ROAD_ID    = "OBJECTID"
COL_CELL_ID    = "id"
COL_POP        = "POP20"
COL_HOUSING    = "HOUSING20"

# Working CRS: UTM 14N (meters), correct for Nebraska distances.
TARGET_EPSG = 32614

# Drone physics (idealized rotor model)
M_DRONE   = 32.0
ZETA      = 5.91
G         = 9.81
RHO       = 1.225
V_DRONE   = 10.0
B_BATTERY = 808 * 3600 * BATT_MULT
T_HOVER   = 45.0
T_INIT_SEC = 120.0

P_DRONE  = (M_DRONE * G) ** 1.5 / math.sqrt(2.0 * RHO * ZETA)
E_FLIGHT = P_DRONE / V_DRONE
E_SQ     = P_DRONE * T_HOVER

# Stop siting coverage radius (scales with battery)
MILE = 1609.34
R_COVER_MILES = 5.0 * BATT_MULT
R_COVER = R_COVER_MILES * MILE

# Demand sampling: log-weighted by population, nested, fixed seed.
DEMAND_SIZES = [5, 10, 20, 30, 40, 50]
SEED = 42
NESTED = True
LOG_OFFSET = 1.756

# Truck speed (mph) by road class; PRIVATE excluded.
SPEED_MPH = {"PRIMARY": 65, "SECONDARY": 55, "LOCAL": 45, "RAMP": 45}
EXCLUDE_CLASSES = {"PRIVATE"}
DEFAULT_SPEED_MPH = 45
MPH_TO_MS = 0.44704

# Truck service and MOVESTAR
T_STOP_MIN = 3.0
T_STOP_SEC = T_STOP_MIN * 60.0
KJ_TO_J = 1000.0
TRUCK_VEH_TYPE = 2   # 2 = light-duty truck (F-150)

# Solver (HiGHS, free)
SOLVER_NAME = "appsi_highs"
SOLVER_EXECUTABLE = None
MIP_GAP = 0.02
TIME_LIMIT = 1800

# Graph endpoint snap tolerance (m)
NODE_SNAP_TOL = 1.0

os.makedirs(OUT_DIR, exist_ok=True)
