"""The Individual data model (genome + metrics) and its serialization for the checkpoint."""
from typing import List, Optional

from aerialist.px4.obstacle import Obstacle

from geometry import _gene_to_obstacle
from testcase import TestCase


class Individual:
    def __init__(self, genes: List[List[float]]):
        self.genes = genes              # list of genes (1..3)
        self.fitness: Optional[float] = None
        self.min_distance: float = float("inf")
        self.completed: bool = False    # did the drone reach the destination?
        self.test_case: Optional[TestCase] = None
        self.fail_base: Optional[str] = None      # base (relative to output_dir) of the fail_* files on disk
        self.avg_point: Optional[float] = None    # average point(sim) (Formula 1) over the runs
        self.avg_flight_min: Optional[float] = None  # average flight duration (minutes), for the ranking
        self.traj: Optional[List[List[float]]] = None  # average (x,y) trajectory at TRAJ_N points, for DTW

    def obstacles(self) -> List[Obstacle]:
        return [_gene_to_obstacle(g) for g in self.genes]


def _ind_to_dict(ind: "Individual") -> dict:
    """Serializes an Individual into JSON-safe data (no aerialist objects)."""
    inf = float("inf")
    return {
        "genes": [[float(v) for v in g] for g in ind.genes],
        "fitness": None if ind.fitness in (None, inf) else float(ind.fitness),
        "min_distance": None if ind.min_distance == inf else float(ind.min_distance),
        "avg_point": ind.avg_point,
        "avg_flight_min": ind.avg_flight_min,
        "completed": bool(ind.completed),
        "fail_base": ind.fail_base,
        "traj": ind.traj,   # needed for DTW after a resume from crash
    }


def _ind_from_dict(d: dict) -> "Individual":
    inf = float("inf")
    ind = Individual([[float(v) for v in g] for g in d["genes"]])
    ind.fitness = inf if d.get("fitness") is None else float(d["fitness"])
    ind.min_distance = inf if d.get("min_distance") is None else float(d["min_distance"])
    ind.avg_point = d.get("avg_point")            # None for old checkpoints -> fallback
    ind.avg_flight_min = d.get("avg_flight_min")  # None for old checkpoints -> neutral time
    ind.completed = bool(d.get("completed"))
    ind.fail_base = d.get("fail_base", d.get("raw_base"))  # raw_base: fallback for old checkpoints
    ind.traj = d.get("traj")   # None on pre-DTW checkpoints -> fallback to IoU
    return ind
