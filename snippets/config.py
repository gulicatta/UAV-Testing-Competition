"""Constants and parameters (env-configurable) for the ES generator."""
import os
import random

from aerialist.px4.obstacle import Obstacle

# ---- Official competition ranges ----
MIN_SIZE = Obstacle.Size(2, 2, 10)
MAX_SIZE = Obstacle.Size(20, 20, 25)
# Legal space: an obstacle outside these limits is invalid
MIN_POS = Obstacle.Position(-40, 10, 0, 0)
MAX_POS = Obstacle.Position(30, 40, 0, 90)

# Sampling range (init/restart): subset of the official limits, concentrated
# around the route to converge faster. Mutation can still leave it, within
# the official limits.
SAMPLE_X_MIN = float(os.environ.get("SAMPLE_X_MIN", -15))
SAMPLE_X_MAX = float(os.environ.get("SAMPLE_X_MAX", 15))
SAMPLE_Y_MIN = float(os.environ.get("SAMPLE_Y_MIN", MIN_POS.y))
SAMPLE_Y_MAX = float(os.environ.get("SAMPLE_Y_MAX", MAX_POS.y))

# ---- ES parameters (env-configurable) ----
# The evaluator launches the generator without env vars: these defaults are
# therefore the algorithm that actually runs in evaluation.
N_RUNS = int(os.environ.get("N_RUNS", 1))
TEST_TIMEOUT = int(os.environ.get("TEST_TIMEOUT", 500))
SIM_RETRIES = int(os.environ.get("SIM_RETRIES", 3))
# Anti-noise confirmation: if a sim drops below CONFIRM_BELOW, repeat the
# evaluation with CONFIRM_EXTRA_RUNS extra runs before trusting the distance.
CONFIRM_BELOW = float(os.environ.get("CONFIRM_BELOW", 1.0))
CONFIRM_EXTRA_RUNS = int(os.environ.get("CONFIRM_EXTRA_RUNS", 1))
# Adaptive sampling: places obstacles along the mission's nominal route
# instead of a fixed box. ADAPTIVE=0 falls back to the fixed box.
ADAPTIVE = os.environ.get("ADAPTIVE", "1") not in ("0", "false", "False", "")
ROUTE_JITTER = float(os.environ.get("ROUTE_JITTER", 3.0))  # std (m) of the jitter around the route
# Mission completion: informational only (does not enter fitness/archive).
# "Reached" = the trajectory passes within DEST_TOLERANCE of the destination
# in at least COMPLETE_MIN_FRAC of the runs.
DEST_TOLERANCE = float(os.environ.get("DEST_TOLERANCE", 5.0))         # m from the destination
COMPLETE_MIN_FRAC = float(os.environ.get("COMPLETE_MIN_FRAC", 0.5))   # fraction of "reached" runs

SIGMA_INIT = float(os.environ.get("SIGMA_INIT", 0.15))  # fraction of the range, initial
SIGMA_MIN = 0.02
SIGMA_MAX = 0.4
SOFT_FAIL = 1.5   # distance below which it's a soft fail (hard fail = collision, dist ~ 0)
STAGNATION_LIMIT = int(os.environ.get("STAGNATION_LIMIT", 15))  # restart after N generations without improvement
INIT_SEEDS = int(os.environ.get("INIT_SEEDS", 5))  # initial seeds evaluated for the starting point
# each restart generates RESTART_CANDIDATES configurations and picks the
# least similar one (min IoU) to the fails already found
RESTART_CANDIDATES = int(os.environ.get("RESTART_CANDIDATES", 12))
# if at restart all y buckets are already covered, with this probability
# intensify on the most severe fail instead of diversifying (see ESGenerator._intensify_individual)
INTENSIFY_PROB = float(os.environ.get("INTENSIFY_PROB", 0.4))
# SEED initializes the sampling PRNG (init/mutations/restart); simulations
# remain stochastic (Gazebo)
_SEED_ENV = os.environ.get("SEED", "").strip()
SEED = int(_SEED_ENV) if _SEED_ENV else None
if SEED is not None:
    random.seed(SEED)
ARCHIVE_CAP = int(os.environ.get("ARCHIVE_CAP", 20))  # delivery archive slots
# average IoU above which two fails are the same test; fallback when the
# trajectory is missing (the primary criterion is DTW, see below)
DUP_IOU = float(os.environ.get("DUP_IOU", 0.95))
Y_BUCKET_SIZE = float(os.environ.get("Y_BUCKET_SIZE", 10))  # y bucket for spatial diversity

# ---- Trajectory-space diversity (official criterion) ----
# The evaluator compares TRAJECTORIES (DTW), not obstacles: two different
# configurations that produce the same avoidance maneuver are the same
# test. DTW_THRESHOLD = mean deviation (m) below which two trajectories are
# considered equal.
DTW_THRESHOLD = float(os.environ.get("DTW_THRESHOLD", 1.5))
TRAJ_N = int(os.environ.get("TRAJ_N", 100))  # points per stored trajectory (DTW is O(n^2))

# outcomes of _maybe_archive
ARCH_NONE = 0        # not a fail
ARCH_NEW = 1         # new trajectory -> archive grew
ARCH_IMPROVED = 2    # same basin, better test -> entry replaced
ARCH_REDUNDANT = 3   # same basin, not better -> discarded

# gene = [l, w, h, x, y, r]
GENE_LOW = [MIN_SIZE.l, MIN_SIZE.w, MIN_SIZE.h, MIN_POS.x, MIN_POS.y, MIN_POS.r]
GENE_HIGH = [MAX_SIZE.l, MAX_SIZE.w, MAX_SIZE.h, MAX_POS.x, MAX_POS.y, MAX_POS.r]

# ---- Mutation step scale (decoupled from the legal bounds) ----
# The mutation step in (x,y) is tuned on the nominal route's corridor
# (see geometry._set_step_scale), not on the whole legal span: otherwise
# every child would jump off the route and the ES would degenerate into
# random search.
POS_STEP_FRAC = float(os.environ.get("POS_STEP_FRAC", 0.6))  # fraction of route extent
POS_STEP_MIN = float(os.environ.get("POS_STEP_MIN", 3.0))    # m, minimum position step
POS_STEP_MAX = float(os.environ.get("POS_STEP_MAX", 8.0))    # m, maximum position step

# ---- Trap-gene: extra obstacle targeted at the escape gap (see geometry._trap_gene) ----
TRAP_PROB = float(os.environ.get("TRAP_PROB", 0.7))  # fraction of the time the added obstacle is targeted (vs random)
TRAP_JITTER = float(os.environ.get("TRAP_JITTER", 1.0))         # m, spread around the gap
TRAP_SPAN_MULT = float(os.environ.get("TRAP_SPAN_MULT", 3.0))   # long side = TRAP_SPAN_MULT * measured gap
TRAP_SIZE_FRAC = float(os.environ.get("TRAP_SIZE_FRAC", 0.35))  # cap on the long side, fraction of the official span
