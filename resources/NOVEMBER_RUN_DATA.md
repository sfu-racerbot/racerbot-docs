# Read a November race recording on your laptop

> **Who this is for:** new club members who want to know what a car actually recorded.
> **Read first:** no robotics background needed; see the [workspace glossary](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/main/docs/glossary.md) for unfamiliar terms.
> **You'll be able to:** find a run and check a published speed request against its messages.
> **Time:** about 10 minutes after the download.

How fast did Car 2 **ask** to go during its November 11 time trial? The answer is in a *bag*: a recording of timestamped messages. A *topic* is a named stream of messages, like `/drive` for requested steering and speed.

**Use a laptop, not a car or its network.** The recording exercise reads files; it sends no commands. You don't need ROS, a simulator, a running dashboard or a Jetson. Don't play the bag into a live ROS system: it contains driving commands.

## 1. Get the recordings

You need Git, Bash, Python 3 with `venv` and internet access for the downloads. Allow several GB for the transfer and extra disk space for the unpacked checkout. The recordings live in the **car workspace**, not this documentation repository.

**Laptop terminal** — start in a folder where you keep projects. For a new copy:

```bash
git clone --branch main --single-branch --depth 1 https://github.com/sfu-racerbot/Racerbot-Car-Workspace.git
cd Racerbot-Car-Workspace
```

If you already have a copy, open a terminal at its root instead. Continue only on a clean `main` branch. If you have local edits, leave them alone and use a new copy.

**Same terminal** — update the workspace, then its pinned dependencies (submodules):

```bash
git pull --ff-only
git submodule update --init --recursive
git rev-parse HEAD
ls -d data/rb2/racerbot_runs/20261111-*
```

**Working when:** you see a revision ID and 12 timestamped directories. Keep the revision ID with your notes. If the list is empty, check that you are at the car workspace root and the pull succeeded.

## 2. Look inside one run

**Same terminal** — choose Car 2 (`rb2`), November 11, 2026, at 13:59:34:

```bash
RUN=data/rb2/racerbot_runs/20261111-135934
ls "$RUN/bag"
cat "$RUN/auto_record.yaml"
ls "$RUN/metadata"
```

| What you see | What it tells you |
|---|---|
| `bag/metadata.yaml` and `bag_0.mcap`, `bag_1.mcap`, `bag_2.mcap` | The index and three parts of **one** recording. The recorder splits large bags at about 50 MB; use every file listed in the index. |
| `auto_record.yaml`, when present | Which program requested motion, how long the recorder ran, and its activity summary. Recording time is not driving time; a late start can miss the fastest part. |
| `metadata/git/`, `metadata/params/`, `metadata/car.yaml`, when present | The code revision and local changes on the car, the settings captured from running programs, and the car's configuration. These can differ from today's defaults. |

**Working when:** the recorder names `/gap_follow_node` under `algorithms` → `/drive`. It steers toward open space in laser scans without needing a map. If a different run has no recorder summary, look at its bag index; a missing summary doesn't mean the car stood still.

Start with the question, then choose the topic:

| Question | Look for messages on |
|---|---|
| What did the algorithm request, and why? | `/drive` (command), `/drive_intent` (algorithm's explanation, when recorded) |
| What did the vehicle report? | `/sensors/core` (VESC motor controller), `/odom` (wheel-based motion estimate), `/steering/measured` (steering estimate) |
| What could the car sense? | `/scan` (laser ranges), IMU streams such as `/imu/data` or `/sensors/imu/raw` (motion sensors) |
| Was the driver holding the enable button? | `/joy` (gamepad buttons, including the left bumper) |

Check `topics_with_message_count` in **this run's** bag index for a positive count. For example, Car 2's `20261111-154204` has no recorded `/drive_intent` messages. Missing data is not a zero measurement.

## 3. Reproduce one number

**Same terminal** — install a file reader in a private Python environment outside the checkout:

```bash
python3 -m venv "$HOME/.venvs/racerbot-bags"
source "$HOME/.venvs/racerbot-bags/bin/activate"
python -m pip install mcap-ros2-support==0.5.7 PyYAML==6.0.2
```

**Working when:** pip reports success or that the packages are already installed. If Python says `venv` is unavailable on Ubuntu, install the `python3-venv` package and retry.

**Same terminal** — copy the whole block. It decodes the speed from every `/drive` message, then averages just the requests **above** 1.5 metres per second:

```bash
python - "$RUN/bag" <<'PY'
import sys
from pathlib import Path
from statistics import mean

import yaml
from mcap_ros2.reader import read_ros2_messages

bag = Path(sys.argv[1])
index = yaml.safe_load((bag / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
speeds = []
for part in index["relative_file_paths"]:
    for sample in read_ros2_messages(str(bag / part), topics=["/drive"]):
        speeds.append(sample.ros_msg.drive.speed)

above = [v for v in speeds if v > 1.5]
if not above:
    sys.exit("No /drive requests above the threshold; check the run and its topic counts.")
print(f"{len(above)} of {len(speeds)} requests; mean above threshold: {mean(above):.2f} m/s")
PY
```

**Expected:** `4219 of 7422 requests; mean above threshold: 2.66 m/s`. This reproduces the **commanded** column in the [race-day report's second correction](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/76db7abecd8c66ca14bbc56d77e82331a08e3cb6/tools/run_analysis/results/2026-11-14-rb2-race-day-agreement.md#second-correction-2026-11-15-the-time-trial-arm-agrees-on-speed-10-optimistic-was-a-measuring-error).

Each qualifying message counts once. This is **not** the car's ground speed, top speed, lap time or a test of the simulator. If a part is missing, stop; a partial bag can give a plausible but wrong average.

## Choose another November run

Look under `data/<car>/racerbot_runs/YYYYMMDD-HHMMSS/`. The table gives the date once and lists the six-digit times. Maps and racing lines (paths to follow) are **outputs**, under `data/<car>/racerbot_auto/`, not bags.

| Car and date (2026) | Where to look and why |
|---|---|
| Car 1, Nov 11 (UVic) | Gap follow: `153409`, `153436`, `153721`, **`155542`**. The last is the multi-car race used in many [run-analysis reports](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/tree/main/tools/run_analysis/results), not an empty-track time trial. |
| Car 1, Nov 16 | Gap follow: `095058`, `095242`, `095652`. Peak forward wheel speeds (`/odom`, field `twist.twist.linear.x`): **5.82, 5.73 and 2.33 m/s**, respectively. The third recorder summary flags `started_late: true`. |
| Car 2, Nov 9 | `20261109-*` recordings from the day the map racer crashed, and `20261109-*` map outputs. Two runs (`190356`, `190617`) contain only a bag. The date alone doesn't identify a crash in a particular bag. |
| Car 2, Nov 11: time trials and race | Gap follow: `135934`, `142036`, `142254`. Map racer (`pure_pursuit`, follows a saved line): `152132`. |
| Car 2, Nov 11: Team A | `ftg`: `135453`, `141227`; `disparity_extender`: `140322`, `142604`, `152844`, `154204`. The last is a roughly 21-minute recording, not 21 minutes of continuous driving. |
| Car 2, Nov 11: mapping | `143337` and `151026` recorded map building. Their saved outputs are `20261111-143428` and `20261111-151125`, respectively. The offline rebuilds use the **run** times: `20261111-143337-rebuilt`, `20261111-151026-rebuilt`. |
| Car 2, Nov 16 | `racerbot_auto/20261116-105215-line` is a saved racing line and map, not a driving recording. |

## Test an algorithm on a mapped course

Include a course the cars actually mapped, not just a generic simulator track. **SLAM** means the car estimates its position and builds a map from sensor readings at the same time. That map is a sensor-based estimate, not an independent survey of the walls.

On November 11, Car 2 recorded laser scans while mapping in `143337` and `151026`. Use these bags to test map building. To test **driving on a known map**, use the later Car 2 race bag `152132` with the map and line it actually loaded:

**Same laptop terminal**, from the car workspace root:

```bash
RACE=data/rb2/racerbot_runs/20261111-152132
COURSE=data/rb2/racerbot_auto/20261111-151026-rebuilt
grep 'yaml_filename:' "$RACE/metadata/params/map_server.yaml"
grep '^[[:space:]]*waypoints_file:' "$RACE/metadata/params/pure_pursuit_node.yaml"
ls "$COURSE"/{map.yaml,map.pgm,raceline_profiled.csv}
```

**Working when:** both recorded paths contain `20261111-151026-rebuilt`. Their old home-directory prefix belongs to the car; use the three local files listed by `ls`. The YAML locates and scales the map image; the CSV gives points to follow and their target speeds.

Don't substitute the live save `151125` just because it came from the same mapping drive. The earlier `143337-rebuilt` kept a map but **did not produce an accepted racing line**.

For a simulator comparison, run the original and changed algorithm on the same mapped course with the same line, start and random seed. For example, compare maximum distance from the line in metres, using the simulated vehicle's actual positions in both runs. Report which map and line you used; a similarly named simulator asset may not match the recorded inputs.

For a recording-only check, compare the algorithm's steering requests on the saved inputs with `/drive` from `152132`. This tests decisions on a path the real car already took, **not** the path a changed algorithm would have driven.

## Using the data for simulator work

Before relying on any [published finding](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/tree/main/tools/run_analysis/results), reproduce that specific measurement from its bag, as in step 3. Confirming one column doesn't verify the rest of a report.

Use **November** bags to fit and check November behavior. October recordings are historical: Car 1's speed gain, steering limits and IMU setup changed. Keep October comparisons separate rather than pooling the months.

For each simulator change, write down the car, run ID, revision, topic/field, units and time window (including where time zero is). Compare the same metric before and after the change.

Reserve a different November run or day to test the fit. For example, Car 2's `152132` was used to select its simulated floor: checking the floor on that same run is not an independent test.

| If you want to… | Use… |
|---|---|
| Compare another simulated drive numerically | **Headless simulation**: run the simulator without a viewer. |
| Watch new simulated sensor readings arrive | **Live simulation in Lichtblick Desktop**: the intended setup sends new readings through an existing Foxglove-compatible bridge to the native viewer's stock panels. This could be on one development machine, or an isolated simulator on the car computer with the viewer on another machine. |
| Inspect what already happened | **Replay**: inspect saved messages from a known run. A recording cannot tell you how a changed controller would drive a new lap. |

These are distinct workflows, not interchangeable viewers. The bag exercise above verifies offline reading only; it does not establish that a live native viewer, remote connection or on-car simulator works. Do not connect a simulation to a live driving system.

Use the uploaded runs rather than waiting for Car 2's missing Nov 6 bags. If a particular claim depends on a missing bag, leave it unverified and choose another question. Keep analysis output outside the workspace and don't commit run data.
