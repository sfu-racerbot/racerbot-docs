# Collect real-car data for simulator calibration

> **Who this is for:** the driver and recorder at the next supervised Car 1 / Car 2 session.
> **Read first:** [Work safely with the club cars](SAFE_TESTING.md), especially the stop procedure.
> **You'll be able to:** measure each car, record whole drives, and check what can be replayed **before packing up**.
> **Time:** budget a half-day for both cars, including setup, instrumentation and copying the recordings. Do Car 1 first if time is short.

**You drive the required tests by hand.** Use the physical Logitech F710 gamepad in teleop (manual control). A teammate starts and stops the recorder. It collects data automatically; it **does not drive**. Only the optional, separately approved controller comparison uses automatic steering.

| Leave with | Why it matters |
|---|---|
| Loaded axle weights, wheelbase and actual front-wheel angles **for each car** | Today's model assumes where the mass sits and how far the wheels turn. |
| Time-aligned wheel angle, ground position and chassis heading through turns | Distinguishes a slow steering mechanism from tyres slipping after the wheels turn. |
| Several separately recorded drives on an unchanged, mapped floor | Car 1 had only **one** evaluation drive usable for mapped path errors; cutting it into more clips doesn't make it independent. |
| If available, a new matched controller/map/lap recording | Better gyro agreement did **not** guarantee better lap times. A human lap is not a controller benchmark. |

**At the track:** [Set up and reserve runs](#1-set-up-the-floor-and-assign-the-runs) → [weigh and instrument](#2-measure-the-parked-car-and-set-up-the-external-reference) → [prove the stop and recorder](#3-start-the-real-car-then-prove-recording-works) → [drive each run](#4-what-the-driver-does-in-each-d-or-h-run) → [check and copy](#5-accept-or-repeat-before-leaving).

## 1. Set up the floor and assign the runs

- [ ] **Crew:** name the driver, the spotter who can cut motor power, and the recorder. One person may log while stationary; nobody types or watches a screen while driving or acting as the sole safety lookout.
- [ ] **Course:** reserve a clear, dry, level area with runoff and no public traffic. One car powered on the driving network at a time. Mark a start/finish line, direction, straight braking lane and two broad turns. Leave walls and LiDAR-visible landmarks fixed. Photograph the layout and measure the course marks.
- [ ] **Kit:** charged batteries, F710 and secure stand. Bring scales and equal-height pads, tape measure, wheel-angle gauge, two chassis markers, calibrated camera(s) or motion capture, sync light, laptop and backup storage. Check disk space on the Jetson **and** cameras with a pilot recording.
- [ ] **Map:** use the matching club map directory: `map.yaml`, its referenced image and the raw course line (`raceline_raw.csv`, x/y in map metres). Mark the physical start pose on it. Walk the boundaries against the map. A similar-looking November track is not a match.
- [ ] If the layout is new, reserve a **setup-only** manual mapping lap after the safety checks below. Follow [manual mapping and saving](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/afda25b14b082b4ea24038a15fa089d4558d63a7/docs/operations.md#building-a-map) and the [localized waypoint recording procedure](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/afda25b14b082b4ea24038a15fa089d4558d63a7/docs/operations.md#1-one-time-per-track-record-a-racing-line). Save the map with basename `map`, its image and line together. Do not build the reference from an evaluation drive.
- [ ] **Write the split before driving.** Make one row per planned whole drive on the [run sheet](#run-sheet-copy-once-per-drive). Example order per car: pilot, D1, H1, D2, H2; add H3 for Car 1 first. D = development; H = sealed evaluation. Each gets a fresh bag and a new start from rest. Alternate roles across battery/time rather than saving only the last battery for H.

**Target:** two usable development drives and at least two new usable holdouts per car; aim for three holdouts on Car 1. These are collection targets, not statistical guarantees. Give replacements a new ID and the **same** role; retain failures and reasons. Never tune on an H run, the November evaluation drives or the October holdouts.

## 2. Measure the parked car and set up the external reference

**Mass and balance — power off, normal race load.** Fit the usual battery in its marked position, body, sensors and any test markers. Measure wheelbase `L` between axle centres in metres.

1. Zero the scales. Keep all four wheels level, at equal height.
2. Weigh the front axle (`F`, kg) and rear axle (`R`, kg). With one scale, swap it and an equal-height pad; both wheels of the axle being weighed must be on the scale.
3. Re-seat the car and repeat three times.

Record every reading and scale resolution. Total mass is `F + R` in kg; distance **forward from the rear axle** to the centre of gravity (CG) is `L × F / (F + R)`. Front-axle-to-CG distance is the remainder. These readings do not measure CG height or yaw inertia. Do not silently substitute the chassis midpoint.

**Actual steering — measure the wheels, not the stick.** Before power-up, align a marked reference with the body centreline and set up a camera or angle gauge that reads **each front wheel** relative to it. Keep hands, tools and loose markers clear of the linkage.

Record straight-ahead zero, units (degrees or radians), left-positive sign and which side is which.

After the stand and stop checks in §3, use teleop with **zero throttle**. On the stand, record centre → small → medium → centre to each side, then repeat in reverse order three times. Hold each position long enough to settle.

Pair actual left/right readings with the **recorded** final steering command, not a guessed joystick angle. Save approach direction to reveal play, and label this run **unloaded**.

For on-floor turns, use a secured, non-contact wheel view or a calibrated rack sensor with a measured rack-travel-to-wheel-angle conversion. Record its clock, sample rate, zero, scale and mounting geometry.

Verify that it sees both directions **while rolling**. For delay, aim for several actual frames per event (e.g. 120 fps gives 8.3 ms per frame before sync error). A static protractor reading cannot measure in-motion delay.

**Independent ground motion — see both where the car points and where it goes.** Fix a camera above the scored area or use motion capture. Put two distinguishable markers along the car, measure their offsets from the rear axle, and keep both visible through left and right turns. Do not mount the only position camera on the car.

1. Survey at least four non-collinear reference points in view and relate them to the map axes; account for marker height when calibrating the image plane. Use a separate known distance and heading to check scale and orientation. Save original frames, timestamps, calibration, frame rate and check residuals. Don't move the camera afterward.
2. Synchronize each external stream to ROS at **both ends** of every run. Use a flash timestamped in the bag and visible in each camera. Without one, film a distinctive series of **unbound** button presses with the car stopped and LB released; match the edges to `/joy`. All loggers must see the event.
3. Record the paired frame and bag times. Check clock offset, drift and uncertainty. Never sync steering to yaw: that would erase the delay being measured. If timing uncertainty is as large as the delay, report **no actuator-lag result**.
4. Check the pilot clip: the marker pair gives chassis heading; the path of the measured CG gives travel direction. Their difference is sideslip. Reject obscured, blurred and near-stationary samples.

**If equipment is missing:** weighing, static steering and command/gyro runs still have value. Mark the affected metric **unmeasured**.

Wheel speed is not ground speed in slip; the servo echo is a command; `/steering/measured` is calculated from motion. Matching laser scans to a map uses a wheel/gyro prediction. It is not external tracking. None replaces missing loaded wheel angle or independent sideslip.

## 3. Start the real car, then prove recording works

Use the club's **already built Jazzy car workspace on the selected Jetson**, not the Humble classroom container. No simulator, test publisher, bag playback, autonomy launch or second car on this graph. No new code/configuration on the floor: any change must first pass the club's [isolated static and wheels-up checks](SAFE_TESTING.md#test-changes-in-this-order).

**Every Jetson terminal — Bash:** use the car's usual ROS network settings, not a simulator domain. Leave car selection to the Jetson serial.

```bash
cd ~/racerbot-ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

**Working when:** both `source` commands return silently. If either file is missing, have the maintainer restore the approved car workspace.

**Terminal A — preflight.** Secure the car on its stand, wheels clear, LB released, F710 switch set to X. Identify the power disconnect. Check the physical car label against the first line of the doctor report:

```bash
git rev-parse HEAD
df -h .
ros2 node list
ros2 run racerbot_doctor doctor
```

**Working when:** the doctor names the correct car and reports `0 FAIL`. If a ROS program is missing, the ID is wrong, or an unexpected driver/control layer is present, stop and resolve it with the maintainer. Do not override the car identity.

**Launch in order**, leaving each process in its own sourced terminal. Do not start two recorders. These commands assume the car still has its approved hardware settings.

| Terminal | Command | What to check before continuing |
|---|---|---|
| A — hardware | `ros2 launch f1tenth_stack bringup_launch.py record:=false` | Drivers and command selector start. The car stays still. This turns off **only** bringup's automatic recorder, which doesn't capture manual driving. |
| B — sensors | `ros2 launch imu_bridge vehicle_state_launch.py` | IMU (motion sensor) starts. Keep the car still at least the first 3 s for gyro bias. |
| C — manual control | `ros2 launch f1tenth_stack teleop_launch.py` | Only teleop controls the car. **LB** (left bumper) must be held to drive. Left stick = speed; right stick = steering. Sticks centred, LB released = no drive. |
| D — checks | `ros2 run racerbot_doctor doctor`, then `ros2 topic echo /imu/status --once`, then `ros2 topic hz /imu/vesc/data` | Correct car, `0 FAIL`, IMU state `ok` and a steady VESC gyro rate. Ctrl+C ends the rate check only. Status describes the selected primary IMU. |

**Terminal E — start one bag (timestamped recording) per run.** Start this with LB released, after the nodes exist so their parameters can be saved. Start all independent loggers too. Do not rely on teleop to trigger recording.

```bash
# Add the actual sensor channel after /imu/status if it publishes to ROS.
TOPICS=$(python3 -c 'from race_diagnostics.recording import DEFAULT_TOPICS; print(" ".join(DEFAULT_TOPICS))') &&
  ros2 launch race_diagnostics record_run.py record_bag:=true topics:="$TOPICS /imu/status"
```

**Working when:** the recorder prints a new directory and subscribes to the live channels. Copy the printed `data/rb1/racerbot_runs/<time>` or `data/rb2/racerbot_runs/<time>` path into the run sheet.

Keep the car stationary while parameters are captured. Record five seconds of still data and the sync event. A camera or wheel sensor outside ROS is **not** saved by this command.

**Pilot on the stand:** with the spotter ready, check steering direction, gently request motion, then release LB. The drive request must go to zero and the wheels stop; verify the actual car. **If it doesn't stop, cut drive power. No floor testing.**

Complete the unloaded steering measurements with the recorder running. Close the stand bag and check it using §5. With the car stopped, lower it safely. Make a **separate floor pilot**: walking-pace stop, then a broad turn each way. Check the external views, angle, speed and gyro against these known motions before collecting D/H runs.

**Abort:** release LB for a person entering, a lost controller link, unexpected motion, inadequate stopping distance or a failed sensor/recorder. Cut power if the car doesn't stop. Never bypass LB, raise a cap to meet a target, force a skid or use a wall as a brake.

## 4. What the driver does in each D or H run

Use the same measured layout and the same approved speed envelope for both splits. Agree on a stopping mark with runoff **before** moving. Start with a crawl; add a second, modest speed only if the maintainer and driver can repeat the stop comfortably within the clear lane. The log records the actual speed and angle; stick positions need not be exact.

| Segment, in order | Driver instruction | What the recorder marks |
|---|---|---|
| Still start | Start at the marked pose, sticks centred; do the sync event. | Run ID, still interval, start pose/direction, battery voltage. |
| Straight / slow / stop | Make three forward passes. Straighten, increase speed gently, hold it steady, then slow to rest before the mark. Include transitions down to a lower speed, not reverse. | Start/end of each phase; commanded vs ground speed, stopping time/distance. |
| Steady left and right | Drive broad constant arcs both ways at steady throttle. Repeat three times per side; use two distinguishable steering magnitudes where space allows. | Side, speed band and angle plateau. Keep the instrumented turn entirely in view. |
| Turn-in and release | From straight at steady low speed, turn to a modest angle, hold, then unwind. Repeat left/right three times; join broad turns into a gentle S. | Command edge, actual wheel onset/settling and gyro response on separate clocks aligned by the sync event. |
| Continuous loop | Drive repeatable full loops on the marked course, aiming for at least two clean line-to-line laps after the start. Use both straights and bends; don't pause in every corner. | Start/finish crossings, lap direction, stops/interventions, contacts and obstructed views. |
| Still finish | Stop in the clear zone, release LB; repeat the sync event and hold still five seconds. | End voltage, final timestamps, changed conditions. Stop the bag and cameras **before** moving the car by hand. |

Aim for several **12-second** moving stretches with visible turns: two seconds of command history plus ten scored. For path comparisons, aim for at least four uninterrupted seconds of trusted ground motion.

These are data goals, **not driving minima**. If the safe speed or space cannot support a test, record the coverage gap. Do not speed up to make a file qualify.

**Optional matched controller lap — separate from the manual plan.** Get the maintainer's approval for this car, course, controller and capped floor-test profile. Follow the [existing supervised race procedure](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/afda25b14b082b4ea24038a15fa089d4558d63a7/docs/operations.md#race-a-mapping-runs-saved-map). Use no new or experimental controller.

End teleop **and** its separate IMU launch first; the race launch supplies its own IMU. Keep the LB operator and power spotter. Recheck safety on the stand and start a fresh recorder with the actual controller parameters captured before motion.

Reserve this whole drive before inspection. Capture the exact controller revision/settings, map, driven line and speed profile, starting pose/speed, direction, independent line-crossing times, contacts and intervention times. Separate standing from flying laps.

Without that match, write **closed-loop transfer not tested**; never compare simulator-controlled laps against hand-driven lap times as if they used the same controller.

## 5. Accept or repeat before leaving

**Terminal E, after each stopped run:** press Ctrl+C once in the recorder and wait for it to finish writing. Keep the whole run directory, not just its first MCAP part. In a sourced terminal:

```bash
read -r -p 'Paste the complete run directory printed by the recorder: ' RUN
ros2 bag info "$RUN/bag"
cat "$RUN/metadata/car.yaml"
ls "$RUN/metadata/params"
```

**Pass the pilot only if** the ID, duration and message counts make sense, external files open, and both sync events are present. Check changing values, not just channel names. Repeat for both cars. Record any missing channel or metadata as a limitation.

| Needed for | Channels/files to inspect |
|---|---|
| Input and safety | `/joy` (LB on/off), `/teleop` and **`/ackermann_cmd`** (final speed in m/s, steering in rad). `/drive` can be empty in manual mode. Never replay it in place of the final manual command. |
| Motor and wheels | `/commands/motor/speed`, `/commands/servo/position`, `/sensors/core`, `/sensors/servo_position_command`, `/odom`. Check commanded steps, wheel-speed changes, voltage and stops. |
| Body turn | `/sensors/imu/raw` (VESC gyro in deg/s) and `/imu/vesc/data` (calibrated car-axis gyro in rad/s); retain `/imu/status` and BNO data if available. Check still bias, sign in both turns, timestamps and gaps. |
| Path / slip / rack | `/scan`, `/tf`, `/tf_static`, matching map and line; calibrated external ground track, heading and actual wheel-angle stream. Confirm useful coverage while moving, not just while parked. |
| Reproducibility | `metadata/` (car, code and parameters), raw scale/angle sheets, map/calibration originals, timestamps, videos/logs, session and run sheets. Preserve ROS header stamps **and** bag receive times. |

**Quality check, not a fit:** count usable seconds per direction. The [November gyro check](CAR2_NOVEMBER_CORNERING.md) used forward corners above 1 m/s wheel speed and at least 0.10 rad of absolute steering command. Count missing bins; do not chase them on the floor.

That check discarded command gaps over 150 ms and wheel/gyro interpolation gaps over 100 ms. Never interpolate through a dropout. If an instrument fails, mark which metrics remain usable.

<details>
<summary>Recorder's laptop: count usable mapped windows without scoring a holdout</summary>

**Before the session — analysis laptop:** prepare the [analysis environment](SLAM_SIM_CALIBRATION.md#try-one-recorded-run) with NumPy, SciPy, Pillow, PyYAML and `rosbags==0.10.11`. Use the [checked workspace revision](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/tree/afda25b14b082b4ea24038a15fa089d4558d63a7).

**At the track — laptop, workspace root:** use a **copy** of the closed run and matching map. This reads files only: no ROS launch, fit or model score.

```bash
read -r -p 'Run directory on laptop: ' RUN
read -r -p 'Matching map directory (map.yaml, image, raceline_raw.csv): ' MAP
read -r -p 'Measured rear axle to CG distance in metres: ' LR
PY="$HOME/.local/racerbot-analysis-venv/bin/python"
OUT=$(mktemp -d)
"$PY" tools/run_analysis/loc.py "$RUN" --map "$MAP" --workers 1 --out "$OUT"
PYTHONPATH=tools:tools/f1tenth_sim "$PY" - "$OUT/$(basename "$RUN").npz" "$LR" <<'PY'
from collections import Counter
import sys
import numpy as np
from run_analysis.slam_motion import extract_motion
from slam_rollout import windows
motion = extract_motion(np.load(sys.argv[1])["a"], float(sys.argv[2]))
counts = Counter(h for h, _, _ in windows(motion))
print({f"{h}s": counts[h] for h in (1, 2, 4)})
PY
```

**Working when:** localization reports matched scans and the dictionary has nonzero 1-, 2- and 4-second counts. Windows start at ≥1 m/s ground speed inside a trusted segment (at least 75% of beams within 10 cm of the map, with jumps and gaps removed).

Check that some windows are in the turns. Zero is **no coverage**, not zero error. Keep an under-speed run for the tasks it can answer.

If no track file appears, check bag, map, image and line. If counts vanish, check scene geometry, time continuity and occlusions on the pilot. A straight-only track does not validate cornering. In teleop, the localization tool's cached command columns read `/drive`; take manual replay inputs from the bag's `/ackermann_cmd` instead.

</details>

**Release checklist:** verify at least two distinct usable Car 1 holdouts, not two windows from one bag. If a run is unusable, keep its ID, reason and files; repeat only after fixing the capture issue. Seal the evaluation files after **coverage-only** checks, before anyone compares a model to them.

At shutdown, release LB, stop the control layer, stop recording and sensor processes cleanly, then stop bringup and power down per [club car operations](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/afda25b14b082b4ea24038a15fa089d4558d63a7/docs/operations.md#shutting-down-cleanly). Verify two private copies of the raw folder and external references by file count and SHA-256 before clearing any device. **Do not commit or push raw bags, video or private data.**

## Run sheet (copy once per drive)

```text
Before motion
  Date/time zone:                 Car:             Run ID + role (pilot / D / H / matched):
  Driver / spotter / recorder:                     Mode, controller and approved limits:
  Workspace commit + local edits:                  Car overlays / map / line / speed profile IDs:
  Layout, surface, start pose and direction:       Tyre set/wear, battery ID/position, payload:
  Geometry sheet (L, F, R, CG, units/errors):      Camera / wheel-angle calibration IDs:
After stopping
  Bag path / external filenames:                   Start / end time and voltage:
  Sync pairs (frame, bag time) start/end:          Offset / drift / uncertainty / frame rate:
  Segment start–end, maneuver, side, speed:        Lap crossings, contacts or interventions:
  Dropouts / occlusions / changes:                 Parameter snapshot and calibration files:
  Core channel counts / usable turn seconds L,R:   Trusted windows 1s / 2s / 4s:
  Usable for / excluded from (reason):             Two private copies / checksum manifest:
```

### Handoff to Claude

1. Give Claude the private bundle location and manifests split by **car and whole-run role**, including failed runs and missing measurements. Fit only on D. Freeze the model, exclusions and acceptance rules before opening H.
2. Replay identical final commands through baseline and candidate. Compare gyro turn error, external ground speed/heading/slip, and 1/2/4-second path error. Separately compare laps and contacts under the matched controller.
3. Report eligible drives and coverage per car. Leave defaults unchanged where a required reference or transfer check is missing. See [the prior tyre result](SLAM_SIM_CALIBRATION.md) and [why the turn-rate experiment stayed optional](GYRO_TURN_RESPONSE.md).
