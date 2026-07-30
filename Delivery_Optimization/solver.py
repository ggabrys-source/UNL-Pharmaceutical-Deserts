# Builds the MIP solver (HiGHS by default) for Pyomo.
import pyomo.environ as pyo
import config as C


def make_solver():
    if C.SOLVER_EXECUTABLE:
        s = pyo.SolverFactory(C.SOLVER_NAME, executable=C.SOLVER_EXECUTABLE)
    else:
        s = pyo.SolverFactory(C.SOLVER_NAME)
    try:
        if "highs" in C.SOLVER_NAME:
            s.options["mip_rel_gap"] = C.MIP_GAP
            s.options["time_limit"] = float(C.TIME_LIMIT)
        elif "cplex" in C.SOLVER_NAME:
            s.options["mipgap"] = C.MIP_GAP
            s.options["timelimit"] = C.TIME_LIMIT
    except Exception:
        pass
    return s
