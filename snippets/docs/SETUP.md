# Setup Guide

**Project:** UAV Test Generator — (1+1) Evolution Strategy  
**Repository branch:** `es-generator`  
**Release date:** 17 July 2026

## 1. Purpose

This guide explains how to install, configure, verify, execute, and
troubleshoot the UAV test generator developed for the UAV Testing
Competition.

The implemented project contains two test-generation methods:

1. a `(1+1)` Evolution Strategy, selected by default;
2. an independent random-search baseline, used for experimental comparison.

The generator executes PX4-Avoidance missions through Aerialist and searches
for obstacle configurations that cause the UAV to violate the required
safety distance.

The recommended execution workflow uses Docker. PX4, PX4-Avoidance, ROS,
Gazebo, and Aerialist are supplied by the `skhatiri/aerialist` Docker image
and therefore do not have to be installed manually on the host.

---

## 2. Execution Architecture

```text
Linux host
│
├── Git repository
│   └── snippets/
│       ├── start.sh
│       ├── cli.py
│       ├── es_generator.py
│       ├── random_generator.py
│       ├── case_studies/
│       └── generated_tests/
│
└── Docker
    └── skhatiri/aerialist
        ├── Aerialist
        ├── PX4-Avoidance
        ├── ROS
        └── Gazebo
```

The `start.sh` launcher mounts the local `snippets/` directory into the
container, copies the generator source files into Aerialist, starts the
simulation, and writes the generated artifacts back to the host.

A host-side Conda environment is not required for this recommended workflow.

---

## 3. Supported Platform and Prerequisites

The recommended host configuration is:

| Component | Requirement |
|---|---|
| Operating system | Linux; Ubuntu is recommended |
| Architecture | x86-64 |
| Version control | Git |
| Container runtime | Docker |
| Simulation image | `skhatiri/aerialist:latest` |
| Shell | Bash |
| Display server | Required only for non-headless execution |

UAV simulations are computationally demanding. Sufficient free memory and
CPU capacity should be available before starting a large campaign.

---

## 4. Clone the Repository

Create the local projects directory:

```bash
mkdir -p ~/Projects
cd ~/Projects
```

Clone only the project branch:

```bash
git clone \
  --branch es-generator \
  --single-branch \
  https://github.com/gulicatta/UAV-Testing-Competition.git
```

Enter the repository:

```bash
cd ~/Projects/UAV-Testing-Competition
```

Verify the selected branch:

```bash
git branch --show-current
```

The expected output is:

```text
es-generator
```

For an existing clone, update it with:

```bash
cd ~/Projects/UAV-Testing-Competition
git fetch origin --prune
git switch es-generator
git pull --ff-only origin es-generator
```

---

## 5. Install Docker on Ubuntu

Update the package index and install Docker:

```bash
sudo apt update
sudo apt install -y docker.io
```

Enable and start the Docker service:

```bash
sudo systemctl enable --now docker
```

Verify the installation:

```bash
docker --version
sudo systemctl status docker --no-pager
```

Allow the current user to execute Docker without `sudo`:

```bash
sudo usermod -aG docker "$USER"
```

Log out of the Linux session and log back in after executing this command.

Then verify Docker access:

```bash
docker run --rm hello-world
```

The project launcher must be able to execute Docker commands without `sudo`.

---

## 6. Pull the Aerialist Image

Download the simulation image:

```bash
docker pull skhatiri/aerialist:latest
```

Verify that the image exists locally:

```bash
docker image inspect skhatiri/aerialist:latest >/dev/null &&
echo "Aerialist image available."
```

List the locally installed image:

```bash
docker images skhatiri/aerialist
```

---

## 7. Repository Location and Portability

The repository may be cloned into any local directory. The launcher determines
the absolute location of the `snippets/` directory from the location of
`start.sh` itself:

```bash
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
```

This resolved directory is used for:

- the `/workspace` Docker mount;
- the local `generated_tests/` directory;
- the Aerialist generated-test output mount.

Therefore, no symbolic link and no fixed repository path are required.

For the repository location used in this guide:

```bash
cd ~/Projects/UAV-Testing-Competition/snippets
```

Verify the portable path configuration:

```bash
grep -nE 'SCRIPT_DIR|/workspace|aerialist/generated_tests' start.sh
```

Verify that the required files are present:

```bash
test -f cli.py
test -f es_generator.py
test -f random_generator.py
test -f testcase.py
test -d case_studies

echo "Repository location verified."
```
## 8. Prepare the Launcher

Enter the generator directory:

```bash
cd ~/Projects/UAV-Testing-Competition/snippets
```

Ensure that the launcher is executable:

```bash
chmod +x start.sh
```

Check its Bash syntax:

```bash
bash -n start.sh
```

A successful syntax check produces no output.

Verify the required generator files:

```bash
for file in \
    cli.py \
    config.py \
    geometry.py \
    models.py \
    es_generator.py \
    random_generator.py \
    testcase.py \
    plot_official.py \
    start.sh
do
    test -f "$file" || {
        echo "Missing required file: $file"
        exit 1
    }
done

echo "All required generator files are present."
```

Verify the available case studies:

```bash
find case_studies -maxdepth 1 -type f -name '*.yaml' -printf '%f\n' | sort
```

---

## 9. Run a Smoke Test

Execute one headless simulation with the default `(1+1)-ES` generator:

```bash
cd ~/Projects/UAV-Testing-Competition/snippets
./start.sh 1 mission1 true
```

The launcher arguments are:

```text
./start.sh <simulation-budget> <mission> <headless>
```

For example:

```text
./start.sh 1 mission1 true
```

means:

- simulation budget: `1`;
- mission: `mission1`;
- headless simulation: enabled.

The first execution can take longer because Docker may need to initialize
the complete PX4, ROS, and Gazebo simulation environment.

---

## 10. Run the `(1+1)` Evolution Strategy

The Evolution Strategy is the default generator.

Run 100 simulations for Mission 1:

```bash
cd ~/Projects/UAV-Testing-Competition/snippets
GENERATOR=es ./start.sh 100 mission1 true
```

Run Mission 2:

```bash
GENERATOR=es ./start.sh 100 mission2 true
```

Run Mission 3:

```bash
GENERATOR=es ./start.sh 100 mission3 true
```

Because `es` is the default value, the following command is equivalent:

```bash
./start.sh 100 mission1 true
```

---

## 11. Run the Random-Search Baseline

Select the random baseline with the `GENERATOR` environment variable:

```bash
cd ~/Projects/UAV-Testing-Competition/snippets
GENERATOR=random ./start.sh 100 mission1 true
```

Examples for the remaining missions:

```bash
GENERATOR=random ./start.sh 100 mission2 true
GENERATOR=random ./start.sh 100 mission3 true
```

The random generator reuses the same:

- legal obstacle space;
- simulation procedure;
- failure definition;
- archive;
- checkpoint representation;
- ranking procedure;
- delivery procedure.

It replaces only the evolutionary search mechanism with independent random
sampling.

---

## 12. Reproducible Search Seeds

Set `SEED` to initialize the generator's pseudorandom sampling process:

```bash
SEED=12345 GENERATOR=es ./start.sh 100 mission1 true
```

For the random baseline:

```bash
SEED=12345 GENERATOR=random ./start.sh 100 mission1 true
```

The same seed reproduces the generator's sampling choices. Complete
simulation outcomes can nevertheless differ because Gazebo and PX4 execution
are not fully deterministic.

---

## 13. Important Runtime Parameters

The runtime parameters listed below are explicitly supported by `start.sh`.
They may be assigned before the command invocation. Variables defined in
`config.py` but not explicitly forwarded by `start.sh` are not propagated
into the Docker container. `MAX_RESTARTS` is handled directly by the
host-side launcher.

| Variable | Default | Meaning |
|---|---:|---|
| `GENERATOR` | `es` | `es` for the `(1+1)-ES`; `random` for the baseline |
| `SEED` | unset | Pseudorandom generator seed |
| `N_RUNS` | `1` | Simulations used for each candidate evaluation |
| `ADAPTIVE` | `1` | Enables nominal-route-based obstacle sampling |
| `INIT_SEEDS` | `5` | Initial candidates evaluated before ES iteration |
| `STAGNATION_LIMIT` | `15` | Generations without improvement before restart |
| `TEST_TIMEOUT` | `500` | Timeout in seconds for one simulation |
| `MAX_RESTARTS` | `12` | Maximum container restarts after failures |
| `DTW_THRESHOLD` | `1.5` | Trajectory-distance threshold for duplicates |
| `TRAJ_N` | `100` | Number of resampled trajectory points |
| `ARCHIVE_CAP` | `20` | Maximum number of archived delivery candidates |
| `TRAP_PROB` | `0.7` | Probability of targeted extra-obstacle placement |
| `INTENSIFY_PROB` | `0.4` | Probability of intensification after coverage |
| `CONFIRM_BELOW` | `1.0` | Distance triggering an additional confirmation |
| `CONFIRM_EXTRA_RUNS` | `1` | Number of additional confirmation simulations |

Example with explicit values:

```bash
SEED=42 \
GENERATOR=es \
INIT_SEEDS=5 \
STAGNATION_LIMIT=15 \
DTW_THRESHOLD=1.5 \
ARCHIVE_CAP=20 \
./start.sh 100 mission1 true
```

To disable adaptive route-based sampling:

```bash
ADAPTIVE=0 GENERATOR=es ./start.sh 100 mission1 true
```

---

## 14. Headless and Graphical Execution

Headless mode is recommended:

```bash
./start.sh 100 mission1 true
```

To display Gazebo graphically:

```bash
sudo apt install -y x11-xserver-utils
./start.sh 100 mission1 false
```

The graphical mode gives the Docker container access to the host X11 display.
Use it only when visual inspection is required.

---

## 15. Checkpoint and Crash Recovery

The generator writes its current search state to:

```text
generated_tests/<run-directory>/checkpoint.json
```

During a single invocation of `start.sh`, a container failure causes the
launcher to restart the container and reuse the same run directory. The
generator then restores:

- consumed simulation budget;
- current parent;
- mutation scale;
- generation number;
- stagnation counter;
- initial-seeding state;
- archived failures;
- nominal route;
- interrupted candidate.

The interrupted candidate is restarted from the beginning rather than from a
partially completed simulation.

The automatic restart loop is limited by `MAX_RESTARTS`:

```bash
MAX_RESTARTS=20 ./start.sh 100 mission1 true
```

Pressing `Ctrl+C` explicitly stops the launcher and prevents another automatic
restart.

A new invocation of `start.sh` creates a new timestamped run directory. It
does not automatically search for a checkpoint created by a previously
terminated launcher invocation.

---

## 16. Generated Output

Each execution creates a directory with the following naming convention:

```text
generated_tests/<mission>-<generator>[-s<seed>]-<date>/
```

Example:

```text
generated_tests/mission1-es-s12345-17-07-14-30-00/
```

The relevant output structure is:

```text
generated_tests/<run>/
├── checkpoint.json
├── debug.txt
├── all_fails/
│   ├── fail_*.yaml
│   ├── fail_*.ulg
│   └── fail_*_plot.png
└── consegna/
    ├── rank01_*.yaml
    ├── rank01_*.ulg
    ├── rank01_*_plot.png
    └── rankNN_*
```

### `all_fails/`

This directory contains every valid safety-distance violation saved during
the campaign, including failures that are later considered redundant or
replaced in the archive.

### `consegna/`

This directory contains the final ranked and diversity-filtered subset.

The files are ordered as:

```text
rank01
rank02
rank03
...
```

where `rank01` is the first test proposed for evaluation.

The `consegna/` directory is the delivery directory. The complete
`all_fails/` history should not be submitted in place of it.

---

## 17. Inspect Generated Results

List all runs:

```bash
cd ~/Projects/UAV-Testing-Competition/snippets
find generated_tests -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | sort
```

List delivered YAML tests:

```bash
find generated_tests -path '*/consegna/rank*.yaml' -print | sort
```

List every saved failure:

```bash
find generated_tests -path '*/all_fails/fail_*.yaml' -print | sort
```

Inspect the log of a specific run:

```bash
less generated_tests/<run-directory>/debug.txt
```

Inspect a checkpoint:

```bash
python3 -m json.tool \
  generated_tests/<run-directory>/checkpoint.json | less
```

---

## 18. Generate or Regenerate Plots

The generator normally creates a plot when it saves a valid failure.

To regenerate plots inside an environment containing the required Python
packages, run:

```bash
python3 plot_official.py \
  generated_tests/<run-directory>/consegna/
```

For the complete failure history:

```bash
python3 plot_official.py \
  generated_tests/<run-directory>/all_fails/
```

For one test:

```bash
python3 plot_official.py \
  generated_tests/<run-directory>/consegna/rank01_<name>.yaml
```

Each YAML file must have a corresponding `.ulg` file.

---

## 19. Self-Contained Docker Build

The repository also provides a `Dockerfile`.

Build the standalone image from `snippets/`:

```bash
cd ~/Projects/UAV-Testing-Competition/snippets
docker build -t uav-es-generator .
```

Run the generator from the built image:

```bash
mkdir -p generated_tests

docker run --rm -it \
  -v "$PWD/generated_tests:/src/generator/generated_tests" \
  uav-es-generator \
  python3 cli.py generate case_studies/mission1.yaml 100
```

The `start.sh` workflow remains preferable for local experimentation because
it provides the project's automatic restart loop and directly mounts the
current source files.

---

## 20. Update the Repository

Before starting new work:

```bash
cd ~/Projects/UAV-Testing-Competition
git fetch origin --prune
git switch es-generator
git pull --ff-only origin es-generator
```

Inspect the state of the repository:

```bash
git status --short --branch
```

Do not pull or switch branches while a simulation is actively writing files.

Generated simulation artifacts should normally remain excluded from commits
unless they are deliberately selected as project evidence.

---

## 21. Common Problems

### Docker permission denied

Symptom:

```text
permission denied while trying to connect to the Docker daemon socket
```

Fix:

```bash
sudo usermod -aG docker "$USER"
```

Log out and log back in, then test:

```bash
docker run --rm hello-world
```

### Docker daemon is not running

Check the service:

```bash
sudo systemctl status docker --no-pager
```

Start it:

```bash
sudo systemctl enable --now docker
```

### Aerialist image is missing

```bash
docker pull skhatiri/aerialist:latest
```

### `/workspace/*.py` cannot be found

Example:

```text
cp: cannot stat '/workspace/testcase.py'
```

The launcher mounts the directory containing `start.sh` as `/workspace`.

Verify that the required project files exist:

```bash
cd ~/Projects/UAV-Testing-Competition/snippets

test -f testcase.py
test -f cli.py
test -f es_generator.py
```

Inspect the resolved directory and Docker mounts:

```bash
grep -nE 'SCRIPT_DIR|/workspace|aerialist/generated_tests' start.sh
```

The expected mount definitions are:

```bash
-v "$SCRIPT_DIR:/workspace"
-v "$SCRIPT_DIR/generated_tests:/src/aerialist/generated_tests"
```

No repository-level symbolic link is required.
### `start.sh` is not executable

```bash
chmod +x start.sh
```

### A previous container remains active

List Aerialist containers:

```bash
docker ps -a --filter ancestor=skhatiri/aerialist
```

Stop and remove them:

```bash
docker ps -aq --filter ancestor=skhatiri/aerialist |
xargs -r docker rm -f
```

### Simulation timeout

The default timeout is 500 seconds. Increase it when necessary:

```bash
TEST_TIMEOUT=700 ./start.sh 100 mission1 true
```

A timeout consumes a simulation attempt because computational resources were
used for that execution.

### LZ4 warning

A warning about the optional Python LZ4 extension does not necessarily mean
that the simulation failed. Determine success from the generator exit status,
the run log, and the generated YAML/ULG artifacts.

### Gazebo or PX4 crashes repeatedly

Inspect the current run log:

```bash
tail -n 100 generated_tests/<run-directory>/debug.txt
```

Check available resources:

```bash
free -h
df -h
docker stats
```

Remove obsolete stopped containers:

```bash
docker container prune
```

Use `docker container prune` only after confirming that no required stopped
container must be retained.

---

## 22. Final Setup Verification

Run the following verification sequence:

```bash
cd ~/Projects/UAV-Testing-Competition/snippets

set -e

command -v git >/dev/null
command -v docker >/dev/null
command -v bash >/dev/null

docker info >/dev/null
docker image inspect skhatiri/aerialist:latest >/dev/null

test -x start.sh
bash -n start.sh

test -f cli.py
test -f config.py
test -f geometry.py
test -f models.py
test -f es_generator.py
test -f random_generator.py
test -f testcase.py
test -f plot_official.py

test -f case_studies/mission1.yaml
test -f case_studies/mission2.yaml
test -f case_studies/mission3.yaml


echo "SETUP CHECK PASSED"
```

The expected final line is:

```text
SETUP CHECK PASSED
```

After this verification, run a one-simulation smoke test:

```bash
./start.sh 1 mission1 true
```
