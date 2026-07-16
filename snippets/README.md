> [!IMPORTANT]
> Before installing or running the project, follow the detailed [Setup Guide](docs/SETUP.md). It contains the required installation, configuration, execution, verification, and troubleshooting instructions.

# UAV Test Generator — (1+1)-ES with DTW diversity

Test generator for the UAV Testing Competition: generates obstacle
configurations that lead the drone (PX4-Avoidance) to violate the safety
distance, maximizing fail severity, trajectory diversity, and simplicity
(few obstacles), per the competition's evaluation criteria.

## Authors

| Name | Affiliation |
|---|---|
| Guglielmo Cattaneo | Politecnico di Milano |
| Roberto Capone | Politecnico di Milano |

## Approach

- **Search**: (1+1) Evolution Strategy. Genome = 1..3 obstacles (`l, w, h, x, y, r`,
  `z` always 0). Gaussian mutation with adaptive sigma (Rechenberg's 1/5 rule).
  Search fitness = minimum drone-obstacle distance (continuous, pushes towards
  collision); the preference for fewer obstacles does not enter the fitness, it
  only lives in the delivery ranking, so as not to distort the search.
- **Adaptive init**: starts from the best of `INIT_SEEDS` random candidates, with
  obstacles sampled along the mission's nominal route (read from the `.ulg`
  next to the yaml; if absent, a bootstrap simulation with no obstacles
  records it) instead of a fixed box — makes the generator independent of
  the mission.
- **Multi-start restart**: after `STAGNATION_LIMIT` generations without
  improvement, restarts from a zone (y bucket) not yet covered by the
  archive, to diversify the fails instead of only refining one. If the
  buckets are already all covered, with probability `INTENSIFY_PROB` it
  intensifies instead: restarts from the most severe fail in the archive to
  refine it further.
- **Trap-gene**: the obstacle added by a mutation is `TRAP_PROB` of the time
  targeted at the point of the parent's trajectory closest to the obstacles
  already present (the escape gap used by the drone), oriented crosswise to
  block it, instead of placed at random — observed that the most severe
  fails found so far all use 2 obstacles in this "trap" configuration.
- **Anti-noise confirmation**: if an evaluation drops below `CONFIRM_BELOW`,
  `CONFIRM_EXTRA_RUNS` extra simulations are spent on the same configuration
  before trusting the distance, so as not to chase a "lucky" fail due to the
  simulator's stochastic noise.
- **Archive and diversity**: obstacles are not the unit of diversity that
  matters — the resulting trajectory is. The archive (slots = `ARCHIVE_CAP`,
  default 20) compares fails by DTW (Dynamic Time Warping) between the
  average trajectories, the same criterion used in evaluation: two fails
  whose trajectory is less than `DTW_THRESHOLD` apart are treated as the
  same test and only one (the best) is kept. A redundant fail counts as
  extra stagnation, so the search leaves an already "milked" basin instead
  of continuing to sample it.
- **Delivery ranking**: `test_score = avg_point * 10 / (#obstacles² * avg_time)`
  — rewards severe fails, few obstacles (quadratic penalty), and short
  flights. Used only to order/select the delivery, never for the search
  fitness.
- **Crash-safety**: incremental checkpoint (`checkpoint.json`) after every
  simulation; a Gazebo crash resumes exactly from the interrupted test
  without losing the archive. `start.sh` wraps the container in an
  auto-resume loop.
- **Random baseline**: `random_generator.py` inherits the entire
  infrastructure (legal space, evaluation, archive, checkpoint, delivery)
  from `ESGenerator` and only replaces the search mechanism with i.i.d.
  sampling, to isolate the evolutionary strategy's contribution in the ES
  vs random comparison.

## Code structure

| File | Role |
|---|---|
| `cli.py` | entry point required by the competition (`generate <mission> <budget>`) |
| `config.py` | constants and parameters (env-configurable) |
| `geometry.py` | obstacle geometry, gene sampling/mutation, nominal route, DTW, scoring formulas |
| `models.py` | `Individual` (genome + metrics) and its serialization for the checkpoint |
| `es_generator.py` | `ESGenerator`: evaluation, checkpoint, (1+1)-ES loop, archive, delivery |
| `random_generator.py` | random search baseline (subclass of `ESGenerator`) |
| `testcase.py` | test execution wrapper (unchanged from the Aerialist template) |
| `plot_official.py` | X/Y/Z/Yaw plot + top-down view for every delivered test |
| `start.sh` | starts a full run in a Docker container, with auto-resume on crash |
| `Dockerfile`, `requirements.txt` | image and dependencies |

Note: diversity (DTW) and noise calibration are not a submission
requirement — the organizing team evaluates them on their own
infrastructure. The part that matters to us is already inside
`es_generator.py` (`_dtw_mean`, `_dup_of`, `_delivery_selection`): it
decides which fails end up in `consegna/`. `DTW_THRESHOLD` is tunable via
env (default 1.5, see table below).

## How to run

### Local development (recommended, with auto-resume on crash)

```bash
./start.sh <budget> <mission> [headless]
# example:
./start.sh 100 mission1
```

`start.sh` mounts `snippets/` into the `skhatiri/aerialist` container (the
base image, no build needed) and copies the code into it on every start: it
launches with `AGENT=local` and automatically restarts from checkpoint if
Gazebo crashes, until the budget is exhausted or the user interrupts with
Ctrl+C.

Main environment variables (all optional, default = configuration
validated in production):

| Variable | Default | Effect |
|---|---|---|
| `GENERATOR` | `es` | `es` = (1+1)-ES, `random` = baseline |
| `N_RUNS` | `1` | simulations per evaluation during the search (screening; the official evaluator runs 5) |
| `SEED` | — | initializes the sampling PRNG, for labelable/repeatable runs |
| `ADAPTIVE` | `1` | sampling along the nominal route; `0` = fixed box (baseline for comparison) |
| `STAGNATION_LIMIT` | `15` | generations without improvement before restart |
| `INIT_SEEDS` | `5` | candidates evaluated for the smart init |
| `TEST_TIMEOUT` | `500` | maximum seconds per single simulation |
| `DTW_THRESHOLD` | `1.5` | DTW distance (m) below which two trajectories are considered the same test |
| `TRAJ_N` | `100` | points per resampled trajectory (used by DTW) |
| `ARCHIVE_CAP` | `20` | archive capacity (= officially evaluated tests) |
| `INTENSIFY_PROB` | `0.4` | prob. of refining the best fail instead of diversifying, when y buckets are covered |
| `TRAP_PROB` | `0.7` | prob. that the added obstacle targets the escape gap instead of random |
| `TRAP_JITTER` | `1.0` | m, spread of the trap-gene around the targeted point |
| `TRAP_SPAN_MULT` / `TRAP_SIZE_FRAC` | `3.0` / `0.35` | trap-gene sizing on the measured gap |
| `CONFIRM_BELOW` | `1.0` | threshold (m) below which an anti-noise confirmation runs |
| `CONFIRM_EXTRA_RUNS` | `1` | extra confirmation runs when `CONFIRM_BELOW` triggers |

### Self-contained build (Dockerfile)

The `Dockerfile` copies the code into the image (unlike `start.sh`, which
mounts the host at runtime): it's for a standalone run, without depending
on the local folder.

```bash
docker build -t uav-es-generator .
docker run uav-es-generator python3 cli.py generate case_studies/mission1.yaml 100
```

### Directly (inside an already running `skhatiri/aerialist` container)

```bash
python3 cli.py generate case_studies/mission1.yaml 100
```

### Output structure

```
generated_tests/<mission>-<generator>[-sSEED]-<date>/
├── checkpoint.json          search state (crash-safe, automatic resume)
├── debug.txt                complete run log
├── all_fails/                every valid fail found, NEVER cleaned up: complete
│                              history for offline inspection, including
│                              redundant/replaced-in-archive fails
└── consegna/                 ONLY the subset to evaluate: rankNN_*.yaml/
                               .ulg/_plot.png in descending quality order
                               (rank01 = best) — this is the folder whose
                               address goes to the evaluators
```

The address of `consegna/` is in the last log line emitted
(`Delivery: N test(s) in <dir>`).

## Results

Final campaigns, budget 100 per mission (`GENERATOR=es`, default
parameters). Raw artifacts (YAML, ULog, plots) are in the corresponding
`generated_tests/<run-id>/consegna/`.

| Mission | Run | Budget used | Valid fails found | Delivered (diversity-filtered) | Most severe fail found | rank01 (delivery order) |
|---|---|---|---|---|---|---|
| mission1 | `mission1-es-15-07-11-34-50` | 100/100 | 16 | 5 | 0.000 m, 3 obstacles — Hard Fail (collision) | 1.403 m, 1 obstacle — Soft Fail |
| mission2 | `mission2-es-14-07-09-40-58` | 100/100 | 21 | 5 | 0.821 m, 1 obstacle — Soft Fail | 0.821 m, 1 obstacle — Soft Fail |
| mission3 | `mission3-es-14-07-00-31-22` | 100/100 | 26 | 6 | 0.460 m, 1 obstacle — Soft Fail | 0.460 m, 1 obstacle — Soft Fail |

"Valid fails found" is the size of `all_fails/` (every distance-violating
scenario simulated); "delivered" is the DTW-diversity-filtered archive
actually copied into `consegna/` — the gap between the two columns is the
archive collapsing near-duplicate trajectories into one entry each (e.g.
mission3: 26 fails found, only 6 kept as trajectory-distinct).

Mission 1 shows why search fitness and delivery ranking are kept separate
(see [Approach](#approach)): the single most severe fail found is an
actual collision (0.000 m, 3 obstacles) — but in `consegna/` it ranks
**last** (`rank05`), because Formula 2's quadratic obstacle penalty
outweighs its severity once flight time and obstacle count are factored
in. `rank01` is instead a simpler, 1-obstacle Soft Fail (1.403 m). Mission2
and mission3 don't show this effect because their most severe fail
happens to already be the simplest one found.

## Documentation

[`docs/SETUP.md`](docs/SETUP.md): installation, execution, verification, and troubleshooting guide.

[`docs/uml/`](docs/uml/): class diagram, execution flow, and sequence diagram
(Mermaid) of `ESGenerator` and the search cycle.
