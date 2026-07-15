"""
(1+1) Evolution Strategy generator for the UAV Testing Competition.

Genome = 1..3 obstacles [l,w,h,x,y,r] (z=0). Fitness = minimum drone-obstacle
distance during the flight (lower = more severe fail); the number of
obstacles does not enter the fitness, only the delivery ranking.

Mechanisms: smart init from several random seeds; gaussian mutation with
adaptive sigma (1/5 rule); restart on stagnation towards an uncovered y
bucket or, if all covered, intensification on the most severe fail;
trap-gene that often targets the parent's escape gap instead of a random
position; anti-noise confirmation with extra runs below a distance
threshold; delivery ranking per the official Formula 2 (rewards severe
fails, few obstacles, short flights); diversity-first slot archive in
trajectory space (DTW); nominal route read from the mission's .ulg (or
obtained via a bootstrap sim if absent).

Every valid fail goes into <run>/all_fails/ (complete history); at the end
of the run the selected subset is copied into <run>/consegna/ as
rankNN_* (rank01 = best).

SEED (env) makes the sampling repeatable; simulations remain stochastic
(Gazebo).

Modularization: config.py (constants), geometry.py (obstacle geometry,
sampling/mutation, route, DTW, scoring formulas), models.py (Individual
and serialization). This file only contains the orchestration: evaluation,
checkpoint, search loop, archive, delivery.
"""
import glob
import json
import logging
import os
import random
import shutil
import signal
from datetime import datetime
from typing import List, Optional

import numpy as np

from aerialist.px4.aerialist_test import AerialistTest
from aerialist.px4.obstacle import Obstacle
from testcase import TestCase

import geometry
from config import (
    ADAPTIVE,
    ARCH_IMPROVED,
    ARCH_NEW,
    ARCH_NONE,
    ARCH_REDUNDANT,
    ARCHIVE_CAP,
    COMPLETE_MIN_FRAC,
    CONFIRM_BELOW,
    CONFIRM_EXTRA_RUNS,
    DTW_THRESHOLD,
    DUP_IOU,
    INIT_SEEDS,
    INTENSIFY_PROB,
    MAX_POS,
    MIN_POS,
    N_RUNS,
    RESTART_CANDIDATES,
    ROUTE_JITTER,
    SAMPLE_Y_MAX,
    SAMPLE_Y_MIN,
    SEED,
    SIGMA_INIT,
    SIGMA_MAX,
    SIGMA_MIN,
    SIM_RETRIES,
    SOFT_FAIL,
    STAGNATION_LIMIT,
    TEST_TIMEOUT,
    TRAP_PROB,
    Y_BUCKET_SIZE,
)
from geometry import (
    _clamp,
    _fit_center,
    _load_route,
    _mutate_gene,
    _non_overlapping_gene,
    _point,
    _random_gene,
    _ranking_score,
    _reached_destination,
    _repair_overlaps,
    _resample_xy,
    _set_step_scale,
    _traj_gap,
    _trap_gene,
)
from models import Individual, _ind_from_dict, _ind_to_dict

logger = logging.getLogger(__name__)


class _SimTimeout(Exception):
    pass


def _alarm_handler(signum, frame):
    raise _SimTimeout()


signal.signal(signal.SIGALRM, _alarm_handler)


class ESGenerator(object):
    # every valid fail ends up in ALL_FAILS_DIR (permanent, never cleaned up);
    # the subset to deliver is COPIED into DELIVERY_DIR with rankNN_* names
    # (see _dump_fail/write_final_selection)
    ALL_FAILS_DIR = "all_fails"
    DELIVERY_DIR = "consegna"
    Y_BUCKET_SIZE = Y_BUCKET_SIZE  # from config.py; exposed on self for _y_bucket/_restart_individual

    def __init__(self, case_study_file: str) -> None:
        self.case_study = AerialistTest.from_yaml(case_study_file)
        self.budget_used = 0
        self.archive: List[Individual] = []   # all fails found
        self._pending = None                  # genes of the test in progress (exact resume after a crash)
        self._seeding = False                 # True during seed init (resumable)
        self._seed_remaining = []             # seeds not yet evaluated (for resume)
        self._seed_best = None                # best seed so far (becomes the parent)
        self.output_dir = os.environ.get(
            "TESTS_FOLDER",
            f"./generated_tests/{datetime.now().strftime('%Y-%m-%d-%H-%M-%S')}/",
        )
        # adaptive sampling: load the mission's nominal route
        geometry._ROUTE_XY = _load_route(case_study_file) if ADAPTIVE else None
        if ADAPTIVE and geometry._ROUTE_XY is None:
            logger.info("Adaptive mode requested but no route available: "
                        "bootstrap (1 sim with no obstacles) on the first generate().")
        # tune the mutation step on the route's corridor
        _set_step_scale(geometry._ROUTE_XY)

    # ---------- evaluation ----------
    def _run_once(self, obstacles: List[Obstacle]) -> Optional["TestCase"]:
        """Runs one simulation with retry (SIM_RETRIES) and timeout (TEST_TIMEOUT).
        Consumes budget on every attempt, even failed ones. None if all
        attempts fail or the budget runs out during the retries."""
        for attempt in range(SIM_RETRIES):
            if self.budget_used >= self.total_budget:
                return None
            tc = TestCase(self.case_study, obstacles)
            self.budget_used += 1
            try:
                signal.alarm(TEST_TIMEOUT)
                try:
                    tc.execute()
                    d = min(tc.get_distances())
                finally:
                    signal.alarm(0)
                tc.min_distance = d
                return tc
            except _SimTimeout:
                signal.alarm(0)
                logger.warning(f"Timeout attempt {attempt+1}/{SIM_RETRIES}")
            except Exception as e:
                signal.alarm(0)
                logger.warning(f"Sim failed attempt {attempt+1}/{SIM_RETRIES}: {str(e)[:80]}")
        return None

    def _evaluate(self, ind: Individual) -> float:
        """Runs the test N_RUNS times (+ any anti-noise confirmation runs,
        see CONFIRM_BELOW) and returns the fitness (observed minimum distance).
        Saves the run's TestCase with the minimum distance."""
        obstacles = ind.obstacles()
        n_obs = len(ind.genes)
        logger.info(
            "  Mission: " + "; ".join(
                f"obstacle {i + 1} (x={o.position.x:.2f}, y={o.position.y:.2f}, "
                f"l={o.size.l:.2f}, w={o.size.w:.2f}, h={o.size.h:.2f}, r={o.position.r:.1f})"
                for i, o in enumerate(obstacles)
            )
        )
        distances = []
        completed_flags = []
        flight_mins = []
        trajs = []          # resampled (x,y) trajectory of each run -> average
        best_tc = None
        best_d = float("inf")

        # runs_planned extends ONCE if the result is already a promising
        # candidate: anti-noise confirmation before trusting the distance
        # (simulations remain stochastic).
        runs_planned = N_RUNS
        run = 0
        while run < runs_planned:
            if self.budget_used >= self.total_budget:
                break
            tc = self._run_once(obstacles)
            run += 1
            if tc is None:
                continue
            d = tc.min_distance
            distances.append(d)
            reached = _reached_destination(tc.trajectory)
            completed_flags.append(reached)
            flight_min = None
            try:
                p = tc.trajectory.positions
                if len(p) >= 2:
                    flight_min = (p[-1].timestamp - p[0].timestamp) / 60e6  # us -> min
                    flight_mins.append(flight_min)
            except Exception:
                pass
            t_xy = _resample_xy(tc.trajectory)
            if t_xy:
                trajs.append(t_xy)
            if d < best_d:
                best_d = d
                best_tc = tc
            # one line per simulation actually run (including confirmation
            # runs), so budget_used in the log never skips a value
            volo = f" | Flight: {flight_min:.1f}min" if flight_min else ""
            logger.info(
                f"  [RUN {self.budget_used}/{self.total_budget}] "
                f"Dist Min: {d:.3f}m | Obs: {n_obs} | Valid (Reached): {reached}{volo}"
            )
            if (len(distances) == N_RUNS and CONFIRM_EXTRA_RUNS > 0
                    and min(distances) < CONFIRM_BELOW):
                runs_planned += CONFIRM_EXTRA_RUNS
                logger.info(
                    f"  Anti-noise confirmation: dist={min(distances):.3f}m < "
                    f"{CONFIRM_BELOW}m -> +{CONFIRM_EXTRA_RUNS} extra runs"
                )

        if not distances:
            ind.fitness = float("inf")
            ind.min_distance = float("inf")
            ind.completed = False
            return ind.fitness

        # fitness = observed minimum distance; the number of obstacles does
        # not enter here, only the delivery ranking (see _ranking_score)
        ind.min_distance = min(distances)
        ind.fitness = ind.min_distance
        # avg_point/avg_flight_min only serve the delivery ranking
        # (Formula 2); the search fitness remains the continuous minimum distance
        ind.avg_point = sum(_point(d) for d in distances) / len(distances)
        ind.avg_flight_min = (sum(flight_mins) / len(flight_mins)) if flight_mins else None
        # average trajectory over the runs: it's the object the evaluator
        # computes DTW on ("average trajectories among the 5 simulations")
        ind.traj = (np.round(np.mean(np.asarray(trajs, dtype=float), axis=0), 3).tolist()
                    if trajs else None)
        # mission completion: informational only, does not enter fitness or
        # archive (a fail is valid if there's a safety violation, regardless
        # of completion)
        completed_frac = sum(completed_flags) / len(completed_flags) if completed_flags else 0.0
        ind.completed = completed_frac >= COMPLETE_MIN_FRAC
        if ind.min_distance < SOFT_FAIL and not ind.completed:
            logger.info(
                f"  VALID fail without reaching the destination (likely Hard Fail/collision): "
                f"dist={ind.min_distance:.2f}m < {SOFT_FAIL}, completed "
                f"{completed_frac*100:.0f}% -> archived (safety violation)"
            )
        ind.test_case = best_tc
        # aggregate note only when more than one run contributed (confirmation
        # triggered): each individual run is already logged above
        if len(distances) > 1:
            logger.info(
                f"  -> aggregate over {len(distances)} runs: "
                f"min={ind.min_distance:.3f}m | Valid (Reached): {ind.completed}"
            )
        return ind.fitness

    # ---------- route bootstrap (missions without a nominal .ulg) ----------
    def _bootstrap_route_if_needed(self):
        """If the nominal route is unavailable (mission without a .ulg,
        typical of evaluation case studies), runs a sim with no obstacles
        and uses the recorded trajectory as the route (costs 1 budget unit).
        Persisted in the checkpoint, so it doesn't repeat after a crash. If
        even the bootstrap fails, the fixed box SAMPLE_X/Y is kept. Must be
        called BEFORE sampling the seeds (which in adaptive mode depend on
        the route)."""
        if not ADAPTIVE or geometry._ROUTE_XY is not None:
            return
        logger.info("Nominal route missing: bootstrapping with 1 sim with no obstacles.")
        traj = None
        for attempt in range(SIM_RETRIES):
            if self.budget_used >= self.total_budget:
                break
            tc = TestCase(self.case_study, [])
            self.budget_used += 1
            try:
                signal.alarm(TEST_TIMEOUT)
                try:
                    traj = tc.execute()
                finally:
                    signal.alarm(0)
                break
            except Exception as e:
                signal.alarm(0)
                logger.warning(
                    f"Bootstrap attempt {attempt+1}/{SIM_RETRIES} failed: {str(e)[:80]}")
                traj = None
        if traj is None or not getattr(traj, "positions", None):
            logger.warning("Route bootstrap unsuccessful: falling back to fixed box SAMPLE_X/Y.")
            return
        goal = traj.positions[-1]  # last point of the nominal flight = destination
        geometry._DESTINATION = (goal.x, goal.y)
        pts = [(p.x, p.y) for p in traj.positions
               if SAMPLE_Y_MIN <= p.y <= SAMPLE_Y_MAX]
        if len(pts) < 2:
            logger.warning("Bootstrap: route empty within the y window; falling back to fixed box.")
            return
        geometry._ROUTE_XY = pts
        _set_step_scale(geometry._ROUTE_XY)
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        logger.info(
            f"Route from bootstrap: {len(pts)} points, "
            f"x[{min(xs):.1f},{max(xs):.1f}] y[{min(ys):.1f},{max(ys):.1f}] "
            f"| destination (x={goal.x:.1f}, y={goal.y:.1f})"
        )

    # ---------- checkpoint for resuming after a crash ----------
    def _checkpoint_file(self) -> str:
        return os.path.join(self.output_dir, "checkpoint.json")

    def save_checkpoint(self):
        """Writes the search state to disk (atomic write via rename), so we
        can resume from here if the simulator crashes. Simple data only: no
        non-serializable aerialist objects."""
        try:
            data = {
                "budget_used": self.budget_used,
                "seed": SEED,
                "total_budget": getattr(self, "total_budget", None),
                "sigma": getattr(self, "sigma", SIGMA_INIT),
                "generation": getattr(self, "generation", 0),
                "stagnation": getattr(self, "stagnation", 0),
                "success_history": getattr(self, "success_history", []),
                "pending": getattr(self, "_pending", None),  # test in progress (for exact resume)
                "seeding": bool(getattr(self, "_seeding", False)),
                "seed_remaining": getattr(self, "_seed_remaining", []),
                "seed_best": _ind_to_dict(self._seed_best) if getattr(self, "_seed_best", None) else None,
                "parent": _ind_to_dict(self.parent) if getattr(self, "parent", None) else None,
                "archive": [_ind_to_dict(i) for i in self.archive],
                # route and destination: after a crash the bootstrap doesn't
                # repeat and the step stays tuned
                "route_xy": ([[float(x), float(y)] for (x, y) in geometry._ROUTE_XY]
                             if geometry._ROUTE_XY else None),
                "destination": (list(geometry._DESTINATION) if geometry._DESTINATION else None),
            }
            os.makedirs(self.output_dir, exist_ok=True)
            path = self._checkpoint_file()
            tmp = path + ".tmp"
            with open(tmp, "w") as f:
                json.dump(data, f)
            os.replace(tmp, path)   # atomic: a crash mid-write doesn't corrupt it
        except Exception as e:
            logger.warning(f"Checkpoint not saved: {str(e)[:100]}")

    def _load_checkpoint(self) -> bool:
        """If a checkpoint exists in output_dir, restores the state and
        returns True (resumed). Otherwise False (clean start)."""
        path = self._checkpoint_file()
        if not os.path.exists(path):
            return False
        try:
            with open(path) as f:
                data = json.load(f)
            self.budget_used = int(data["budget_used"])
            self.sigma = float(data.get("sigma", SIGMA_INIT))
            self.generation = int(data.get("generation", 0))
            self.stagnation = int(data.get("stagnation", 0))
            self.success_history = list(data.get("success_history", []))
            self._pending = data.get("pending", None)   # interrupted test to redo
            self._seeding = bool(data.get("seeding", False))
            self._seed_remaining = data.get("seed_remaining", [])
            self._seed_best = _ind_from_dict(data["seed_best"]) if data.get("seed_best") else None
            self.archive = [_ind_from_dict(d) for d in data.get("archive", [])]
            self.parent = _ind_from_dict(data["parent"]) if data.get("parent") else None
            # route from the checkpoint: also restore the step tuning,
            # otherwise the resume would run with GENE_STEP at the full span
            route = data.get("route_xy")
            if route and geometry._ROUTE_XY is None:
                geometry._ROUTE_XY = [(float(x), float(y)) for x, y in route]
                _set_step_scale(geometry._ROUTE_XY)
                logger.debug(f"Route restored from checkpoint: {len(geometry._ROUTE_XY)} points")
            dest = data.get("destination")
            if dest and geometry._DESTINATION is None:
                geometry._DESTINATION = (float(dest[0]), float(dest[1]))
            logger.info(
                f"RESUMED from checkpoint: budget_used={self.budget_used}, "
                f"gen={self.generation}, archive={len(self.archive)}"
            )
            # resume is valid if there's a parent (ES already started) or if
            # we're still in the seeding phase (crash during initial seeds)
            return (self.parent is not None) or self._seeding
        except Exception as e:
            logger.warning(f"Checkpoint unreadable, starting over: {str(e)[:100]}")
            return False

    def _ranked_archive(self) -> List["Individual"]:
        """Archive sorted by descending _ranking_score (rank01 = best)."""
        return sorted(self.archive, key=_ranking_score, reverse=True)

    # ---------- (1+1)-ES ----------
    def _eval_seed(self, seed: "Individual"):
        """Evaluates a seed, archives it if it's a valid fail, updates the
        best seed so far (will become the ES's initial parent)."""
        self._evaluate(seed)
        if seed.test_case is not None:
            self._maybe_archive(seed)
        f = seed.fitness if seed.fitness is not None else float("inf")
        best_f = (self._seed_best.fitness if (self._seed_best is not None
                  and self._seed_best.fitness is not None) else float("inf"))
        if f < best_f:
            self._seed_best = seed

    def generate(self, budget: int) -> List["Individual"]:
        self.total_budget = budget
        if not self._load_checkpoint():
            logger.debug(f"Starting (1+1)-ES, budget={budget}, N_RUNS={N_RUNS}, INIT_SEEDS={INIT_SEEDS}")
            # the route is needed before sampling the seeds in adaptive mode
            self._bootstrap_route_if_needed()
            # resumable init: the seeds are evaluated one at a time, each
            # protected by a checkpoint, so a crash resumes from the right seed
            self._seeding = True
            self._seed_remaining = [[_random_gene()] for _ in range(INIT_SEEDS)]
            self._seed_best = None
            self.sigma = SIGMA_INIT
            self.success_history = []   # for the 1/5 rule
            self.stagnation = 0
            self.generation = 0
            self.parent = None
            self.save_checkpoint()
        else:
            logger.debug(f"Starting (1+1)-ES in RESUME mode, budget={budget}")

        # ---- seeding phase (resumable, crash-safe per single seed) ----
        if getattr(self, "_seeding", False):
            # a seed was in progress at the moment of the crash: redo it from scratch
            if getattr(self, "_pending", None) is not None and self.budget_used < self.total_budget:
                s = Individual([list(g) for g in self._pending])
                self._pending = None
                self._eval_seed(s)
                self.save_checkpoint()
            # evaluate the remaining seeds, one at a time, with a checkpoint before and after
            while self._seed_remaining and self.budget_used < self.total_budget:
                genes = self._seed_remaining[0]
                self._seed_remaining = self._seed_remaining[1:]   # removed FIRST: no double evaluation on resume
                s = Individual([list(g) for g in genes])
                self._pending = s.genes                            # mark as in progress
                self.save_checkpoint()                             # checkpoint BEFORE evaluating
                self._eval_seed(s)
                self._pending = None
                self.save_checkpoint()                             # seed completed
            # seeding done: the parent is the best seed (or a random gene if the budget is already exhausted)
            self._seeding = False
            self.parent = self._seed_best if self._seed_best is not None else Individual([_random_gene()])
            self.save_checkpoint()

        while self.budget_used < self.total_budget:
            self.generation += 1

            # test interrupted by a crash: redone from scratch (never
            # halfway); otherwise generate a new child from the parent
            if getattr(self, "_pending", None) is not None:
                child = Individual([list(g) for g in self._pending])
                logger.info(f"Gen {self.generation}: redoing from scratch the test interrupted by the crash")
            else:
                child = self._make_child(self.parent, self.sigma)

            # saved BEFORE evaluating: if Gazebo crashes during _evaluate,
            # on resume this identical test starts over from scratch
            self._pending = child.genes
            self.save_checkpoint()

            self._evaluate(child)
            self._pending = None   # evaluation complete: no test pending anymore

            cf = child.fitness if child.fitness is not None else float("inf")
            pf = self.parent.fitness if self.parent.fitness is not None else float("inf")
            improved = cf < pf
            self.success_history.append(1 if improved else 0)

            if improved:
                self.parent = child
                self.stagnation = 0
            else:
                self.stagnation += 1

            arch = ARCH_NONE
            if child.test_case is not None:
                arch = self._maybe_archive(child)

            # a redundant fail (same trajectory basin, not better) = budget
            # spent without information: extra stagnation to leave the
            # already-milked basin sooner instead of producing more copies
            if arch == ARCH_REDUNDANT:
                self.stagnation += 1

            self.sigma = self._adapt_sigma(self.sigma, self.success_history)

            if self.stagnation >= STAGNATION_LIMIT and self.budget_used < self.total_budget:
                self.parent = self._restart_individual()
                self.sigma = SIGMA_INIT
                self.stagnation = 0
                self.success_history = []
                logger.info(
                    f"  [RUN {self.budget_used}/{self.total_budget}] RESTART "
                    f"(stagnation {STAGNATION_LIMIT} gen) -> new y region {self._y_bucket(self.parent)}"
                )

            self.save_checkpoint()  # checkpoint at the end of the generation

        return self._ranked_archive()

    def _restart_candidate_iou(self, gene) -> float:
        """Maximum IoU of the candidate gene against all obstacles already in
        the archive; lower = newer configuration."""
        try:
            cand = Individual([gene]).obstacles()[0].geometry
        except Exception:
            return 1.0
        worst = 0.0
        for ind in self.archive:
            for ob in ind.obstacles():
                inter = cand.intersection(ob.geometry).area
                union = cand.union(ob.geometry).area
                if union > 0:
                    worst = max(worst, inter / union)
        return worst

    def _restart_individual(self) -> Individual:
        """Restarts from a configuration dissimilar to those already found:
        generates RESTART_CANDIDATES configurations (preferring uncovered y
        buckets, staying on the route) and picks the one with minimum IoU
        against the archive. If all buckets are covered, with probability
        INTENSIFY_PROB intensifies instead on the most severe fail in the
        archive (see _intensify_individual) rather than sampling already-seen
        zones."""
        n_buckets = int((MAX_POS.y - MIN_POS.y) / self.Y_BUCKET_SIZE)
        covered = {self._y_bucket(ind) for ind in self.archive}
        empty_buckets = [b for b in range(n_buckets) if b not in covered]

        def _candidate():
            if empty_buckets:
                target_bucket = random.choice(empty_buckets)
                y_lo = MIN_POS.y + target_bucket * self.Y_BUCKET_SIZE
                y_hi = min(y_lo + self.Y_BUCKET_SIZE, MAX_POS.y)
                gene = _random_gene()
                if ADAPTIVE and geometry._ROUTE_XY:
                    # stay ON the route: a point of the path inside the target y bucket
                    in_bucket = [p for p in geometry._ROUTE_XY if y_lo <= p[1] < y_hi]
                    if in_bucket:
                        rx, ry = random.choice(in_bucket)
                        gene[3] = _clamp(rx + random.gauss(0, ROUTE_JITTER), MIN_POS.x, MAX_POS.x)
                        gene[4] = _clamp(ry + random.gauss(0, ROUTE_JITTER), y_lo, y_hi)
                    else:
                        gene[4] = random.uniform(y_lo, y_hi)  # route doesn't pass here: force y into the bucket
                else:
                    gene[4] = random.uniform(y_lo, y_hi)  # force y into the empty bucket
            else:
                gene = _random_gene()
            return _fit_center(gene)

        # empty archive: nothing to diversify against, just one candidate
        if not self.archive:
            return Individual([_candidate()])
        if not empty_buckets and random.random() < INTENSIFY_PROB:
            return self._intensify_individual()
        # generate several candidates and keep the one least similar to the archive (min IoU)
        candidates = [_candidate() for _ in range(RESTART_CANDIDATES)]
        best_gene = min(candidates, key=self._restart_candidate_iou)
        return Individual([best_gene])

    def _intensify_individual(self) -> "Individual":
        """Restarts from the most severe fail in the archive (by min_distance,
        not _ranking_score which also weighs obstacles/time) + a trap-gene if
        there's room for another obstacle."""
        best = min(self.archive, key=lambda i: i.min_distance)
        genes = [list(g) for g in best.genes]
        if len(genes) < 3 and best.traj:
            extra = _trap_gene(best, genes) or _non_overlapping_gene(genes)
            if extra is not None:
                genes.append(extra)
        genes = _repair_overlaps(genes)
        logger.debug(
            f"Restart: intensifying on the best entry in the archive (d={best.min_distance:.3f}m)"
        )
        return Individual(genes)

    def _make_child(self, parent: Individual, sigma: float) -> Individual:
        new_genes = [_mutate_gene(g, sigma) for g in parent.genes]
        # with small probability add/remove an obstacle (max 3, min 1)
        roll = random.random()
        if roll < 0.10 and len(new_genes) < 3:
            # if the parent has a trajectory, TRAP_PROB of the time the
            # added obstacle targets the escape gap (see _trap_gene) instead of random
            extra = None
            if parent.traj and random.random() < TRAP_PROB:
                extra = _trap_gene(parent, new_genes)
            if extra is None:
                extra = _non_overlapping_gene(new_genes)
            if extra is not None:
                new_genes.append(extra)
        elif roll > 0.90 and len(new_genes) > 1:
            new_genes.pop(random.randrange(len(new_genes)))
        new_genes = _repair_overlaps(new_genes)  # mutation may have created overlaps
        return Individual(new_genes)

    def _adapt_sigma(self, sigma: float, history: List[int]) -> float:
        """Rechenberg's 1/5 rule over a window of min(10, STAGNATION_LIMIT):
        if >1/5 of recent attempts improve, increase sigma (explore more),
        otherwise reduce it (refine)."""
        window_size = min(10, STAGNATION_LIMIT)
        window = history[-window_size:]
        if len(window) < window_size:
            return sigma
        success_rate = sum(window) / len(window)
        if success_rate > 0.2:
            sigma *= 1.22
        else:
            sigma /= 1.22
        return _clamp(sigma, SIGMA_MIN, SIGMA_MAX)

    # ---------- archive & selection ----------
    def _y_bucket(self, ind: Individual) -> int:
        """y bucket (width Y_BUCKET_SIZE) of the obstacle whose x is closest
        to 0 (the most central to the route), relative to MIN_POS.y."""
        best_gene = min(ind.genes, key=lambda g: abs(g[3]))  # g[3] = x
        n_buckets = max(1, int((MAX_POS.y - MIN_POS.y) / self.Y_BUCKET_SIZE))
        b = int((best_gene[4] - MIN_POS.y) // self.Y_BUCKET_SIZE)  # g[4] = y
        return min(max(b, 0), n_buckets - 1)  # y = exact MAX -> last band

    def _plot_fail(self, ulg_path, yaml_path, out_png):
        """Official-style plot of the fail (lazy import: if plot_official is
        missing or the plot fails, the run doesn't stop)."""
        try:
            import contextlib, io
            import plot_official
            # suppresses the summary line plot_official prints to stdout
            with contextlib.redirect_stdout(io.StringIO()):
                plot_official.plot_one(
                    ulg_path, yaml_path, out_png,
                    title=os.path.basename(out_png).replace("_plot.png", ""),
                )
        except Exception as e:
            logger.warning(f"Fail plot unsuccessful: {str(e)[:80]}")

    def _dump_fail(self, ind: Individual):
        """Saves every valid fail (.yaml + .ulg + plot if available) into
        <run>/all_fails/, as soon as the sim finishes and before the next one
        (which could crash Gazebo). Permanent folder, never cleaned up:
        write_final_selection() copies the subset to deliver from here.
        ind.fail_base (path relative to output_dir) is serialized in the
        checkpoint, so delivery stays possible even after a resume from a
        crash (test_case lost)."""
        tc = getattr(ind, "test_case", None)
        if tc is None:
            return  # fail reloaded from checkpoint: its file is already on disk
        all_fails_dir = os.path.join(self.output_dir, self.ALL_FAILS_DIR)
        os.makedirs(all_fails_dir, exist_ok=True)
        d = ind.min_distance
        n = len(ind.genes)
        base = os.path.join(all_fails_dir, f"fail_d{d:06.3f}_obs{n}_run{self.budget_used:03d}")
        try:
            tc.save_yaml(base + ".yaml")
            ind.fail_base = os.path.relpath(base, self.output_dir)  # "all_fails/fail_..."
            ulg = getattr(tc, "log_file", None)
            if ulg and os.path.exists(ulg):
                shutil.copy2(ulg, base + ".ulg")
                self._plot_fail(base + ".ulg", base + ".yaml", base + "_plot.png")
        except Exception as e:
            logger.warning(f"Fail dump unsuccessful: {str(e)[:80]}")

    def _similar_pair(self, a: Individual, b: Individual) -> bool:
        """Same diversity class if same y bucket or similar footprints
        (average IoU >= 0.5). Used for archive eviction."""
        return self._y_bucket(a) == self._y_bucket(b) or self._too_similar(a, b)

    def _replace_in_archive(self, victim: Individual, ind: Individual):
        """In-memory replacement: ind is already on disk in all_fails/. The
        victim's file is left there forever, still inspectable."""
        self.archive.remove(victim)
        self.archive.append(ind)

    def _dup_of(self, ind: Individual) -> Optional[Individual]:
        """Archive entry the evaluator would consider the same test
        (trajectory within DTW_THRESHOLD, the closest one). Falls back to
        IoU (DUP_IOU) when the trajectory is missing (old checkpoints)."""
        best, best_gap = None, float("inf")
        for existing in self.archive:
            gap = _traj_gap(ind, existing)
            if gap is None:   # no trajectory: only comparison possible
                if self._too_similar(ind, existing, iou_thr=DUP_IOU):
                    return existing
                continue
            if gap < best_gap:
                best, best_gap = existing, gap
        return best if best_gap < DTW_THRESHOLD else None

    def _maybe_archive(self, ind: Individual) -> int:
        """Archives valid fails (dist < SOFT_FAIL) with a slot policy
        (ARCHIVE_CAP) that's diversity-first in trajectory space:
        1) trajectory within DTW_THRESHOLD of an entry -> it's the same test
           for the evaluator: only the most severe by min_distance stays;
        2) new trajectory and a free slot -> it enters;
        3) cap full -> it enters only if it beats the lowest score
           (_ranking_score) in the archive.
        Returns the outcome as a novelty signal for the search."""
        if ind.min_distance >= SOFT_FAIL:
            return ARCH_NONE
        # every valid fail goes to disk right away, regardless of the decision below
        self._dump_fail(ind)
        # (1) same trajectory basin: whoever is objectively more severe wins
        # (lower min_distance), not whoever has the better _ranking_score
        dup = self._dup_of(ind)
        if dup is not None:
            if ind.min_distance < dup.min_distance:
                self._replace_in_archive(dup, ind)
                logger.info(
                    f"  fail in the same basin but MORE SEVERE "
                    f"({dup.min_distance:.3f}m -> {ind.min_distance:.3f}m) -> replaced"
                )
                return ARCH_IMPROVED
            logger.info(
                f"  REDUNDANT fail: trajectory already in the archive (DTW < {DTW_THRESHOLD} m) "
                f"and not more severe ({ind.min_distance:.3f}m >= {dup.min_distance:.3f}m) "
                f"-> not entering delivery (stays in all_fails/)"
            )
            return ARCH_REDUNDANT
        # (2) new trajectory, free slot
        if len(self.archive) < ARCHIVE_CAP:
            self.archive.append(ind)
            return ARCH_NEW
        # (3) cap full: enters only if it beats the worst one
        victim = min(self.archive, key=_ranking_score)
        if _ranking_score(ind) <= _ranking_score(victim):
            return ARCH_REDUNDANT
        self._replace_in_archive(victim, ind)
        return ARCH_NEW

    def _too_similar(self, a: Individual, b: Individual, iou_thr: float = 0.5) -> bool:
        """True if the average IoU of the bounding boxes of a and b exceeds the threshold."""
        obs_a = a.obstacles()
        obs_b = b.obstacles()
        ious = []
        for oa in obs_a:
            best = 0.0
            for ob in obs_b:
                inter = oa.geometry.intersection(ob.geometry).area
                union = oa.geometry.union(ob.geometry).area
                if union > 0:
                    best = max(best, inter / union)
            ious.append(best)
        if not ious:
            return False
        return (sum(ious) / len(ious)) >= iou_thr

    def _delivery_selection(self):
        """Subset of the archive to deliver, separated in trajectory space
        (no pair within DTW_THRESHOLD): greedy on descending score, keeping
        only those at least DTW_THRESHOLD away from all the ones already
        kept. A near-duplicate lowers sim_pen (multiplicative over the whole
        suite): delivering fewer but separated tests can be worth more than
        delivering more with clones."""
        keep, dropped = [], []
        for ind in self._ranked_archive():
            gaps = [_traj_gap(ind, k) for k in keep]
            if any(g is not None and g < DTW_THRESHOLD for g in gaps):
                dropped.append(ind)
            else:
                keep.append(ind)
        if dropped:
            logger.info(
                f"  Delivery: {len(dropped)} test(s) dropped for being too similar "
                f"(DTW < {DTW_THRESHOLD} m) to one already selected -> stay in "
                f"{self.ALL_FAILS_DIR}/, not copied into {self.DELIVERY_DIR}/"
            )
        return keep, dropped

    def write_final_selection(self) -> int:
        """Copies the subset chosen by _delivery_selection from
        <run>/all_fails/ to <run>/consegna/ with rankNN_* names (rank01 =
        best, order of descending _ranking_score). Copies rather than
        moving: the sources stay intact in all_fails/, so consegna/ is
        always reconstructible. Idempotent and crash-safe: ranks already
        present are kept, those no longer expected are cleaned up from
        consegna/ (never from all_fails/). Emits the final line required by
        the submission."""
        consegna_dir = os.path.join(self.output_dir, self.DELIVERY_DIR)
        os.makedirs(consegna_dir, exist_ok=True)
        expected = set()     # basenames of the ranks expected from this delivery
        written = 0
        delivered, _dropped = self._delivery_selection()
        for i, ind in enumerate(delivered, start=1):
            base_out = f"rank{i:02d}_d{ind.min_distance:06.3f}_obs{len(ind.genes)}"
            expected.add(base_out)
            dst = os.path.join(consegna_dir, base_out)
            src = (os.path.join(self.output_dir, ind.fail_base)
                   if ind.fail_base else None)
            try:
                if src and os.path.exists(src + ".yaml"):
                    for ext in (".yaml", ".ulg", "_plot.png"):
                        if os.path.exists(src + ext):
                            shutil.copy2(src + ext, dst + ext)
                    written += 1
                elif os.path.exists(dst + ".yaml"):
                    written += 1   # already delivered by a previous run (idempotence)
                elif ind.test_case is not None:
                    ind.test_case.save_yaml(dst + ".yaml")
                    ulg = getattr(ind.test_case, "log_file", None)
                    if ulg and os.path.exists(ulg):
                        shutil.copy2(ulg, dst + ".ulg")
                        self._plot_fail(dst + ".ulg", dst + ".yaml",
                                        dst + "_plot.png")
                    written += 1
                else:
                    logger.warning(f"rank{i:02d}: no files on disk and no test_case; skipping")
            except Exception as e:
                logger.warning(f"Delivery of rank{i:02d} failed: {str(e)[:80]}")

        def _base(name: str) -> str:
            if name.endswith("_plot.png"):
                return name[: -len("_plot.png")]
            return os.path.splitext(name)[0]

        # cleanup: rank* in consegna/ no longer expected (e.g. the DTW
        # threshold changed mid-campaign). all_fails/ is never touched.
        for path in glob.glob(os.path.join(consegna_dir, "rank*")):
            if _base(os.path.basename(path)) not in expected:
                try:
                    os.remove(path)
                except OSError:
                    pass
        logger.info(f"Delivery: {written} test(s) in {consegna_dir}")
        return written
