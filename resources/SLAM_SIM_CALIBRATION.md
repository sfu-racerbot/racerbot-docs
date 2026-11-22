# Does the simulator drive like the real car?

> **Who this is for:** club members who can run Python and want to understand or repeat the November 2026 real-car comparison.
> **Read first:** the [simulator introduction](F1TENTH_GYM_ROS.md) for the idea of simulated sensors and commands.
> **You'll be able to:** follow the measurement pipeline and explain which calibration changes passed, which failed, and what remains uncertain.
> **Time:** about 10 minutes to read; replaying the recorded drives takes several minutes with Gym already installed.

## What changed, in plain words

The wheels can report more speed than the car actually makes over the floor. A simulator that agrees with the wheels can therefore disagree with the real motion.

We now match recorded laser scans to a saved map, measure the motion between those matches, and replay the recorded driving commands in the simulator. This measures **ground speed**, **yaw rate** (how fast the car turns), and **sideslip** (the angle between where the car points and where it travels).

- Both cars' wheels over-read ground speed: about **7.5% for rb1**, **2.6% for rb2** on November development drives.
- Multi-second replays expose speed, turning and position errors, not just whether a simulated lap finishes.
- New tyre fits failed the agreed evaluation rules for **both cars**. They were kept as rejected experiments, not made defaults.
- A separate speed-scale change passed for **rb2 only**. This is a small improvement on November drives, not a claim that every floor is solved.

This page describes the newer **Racerbot-Car-Workspace (ROS 2 Jazzy)** analysis, not a replacement for the older Humble classroom simulator instructions. The experiments run offline: **no physical car moves**.

The [workspace's detailed November report](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/d77bd05c0d0339566ba3f5f21202413b5dd14394/docs/sim-calibration.md#measuring-the-real-car-from-slam-localization-november-2026) is the source for the results below. The link is pinned to the documented revision so its numbers remain reproducible.

## The three-stage pipeline

1. **`loc.py`: where was the car?** Match each recorded LiDAR scan (one sweep of the laser) against the saved map. SLAM means simultaneous localization and mapping; here we localize against an existing map rather than build a new one.
2. **`slam_motion.py`: how did it move?** Smooth the trusted positions and headings, then compute speed, yaw rate and sideslip. Poor matches, pose jumps and gaps are removed; smoothing never crosses a rejected stretch.
3. **`slam_rollout.py`: can the simulator reproduce it?** Start a fresh simulated car from the measured state, feed the same delayed commands, and compare the next **1, 2 and 4 seconds**. This is *open loop*: the simulator does not steer itself back toward the recorded path.

### Try one recorded run

**One analysis terminal, inside the Racerbot-Car-Workspace checkout, not this docs repository.** First follow that workspace's [Gym setup](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/d77bd05c0d0339566ba3f5f21202413b5dd14394/docs/simulator.md). You also need the recorded run and map shown below, and its analysis Python environment with `rosbags==0.10.11` installed; recordings are not supplied by this docs repository.

```bash
PY="$HOME/.local/racerbot-analysis-venv/bin/python"
OUT="$HOME/.cache/racerbot"
mkdir -p "$OUT/loc" "$OUT/slam_motion" "$OUT/slam_rollout"
RUN=data/rb2/racerbot_runs/20261111-152132
MAP=data/rb2/racerbot_auto/20261111-151026-rebuilt
"$PY" tools/run_analysis/loc.py "$RUN" --map "$MAP" --workers 1 --out "$OUT/loc"
"$PY" tools/run_analysis/slam_motion.py "$OUT/loc/20261111-152132.npz" \
  --car rb2 --json "$OUT/slam_motion/example.json" --samples
export PYTHONPATH=.sim/python:.sim/f1tenth_gym:src/gap_follow:src/pure_pursuit:tools/f1tenth_sim:tools
export NUMBA_CACHE_DIR=.sim/numba-cache
"$PY" tools/f1tenth_sim/slam_rollout.py "$RUN" --car rb2 \
  --loc "$OUT/loc" --json "$OUT/slam_rollout/example.json"
```

**Working when:** localization prints the scan count and match quality, and the next two tools write JSON reports under the cache directory. No output needs to go into `data/`.

The bag reader uses ROS readers when available, or falls back to `rosbags.highlevel.AnyReader` on the bag directory without ROS. A missing `rosbags` import means the analysis environment is incomplete; a missing Gym import means the simulator setup is incomplete.

Localization trust requires at least 75% of beams within 10 cm of the map. Rollout starts are at least one second apart, at ground speed of at least 1 m/s; each whole window must stay in one trusted stretch. Each gets a fresh seeded plant so simulator memory cannot leak between windows.

## Which runs were used?

**Development** runs provide measurements and fit parameters. **Evaluation** runs judge the candidate only after its values and acceptance rule are frozen. Names below are `date-time` directory names, abbreviated by date.

| Car/date | Development times | Evaluation times |
|---|---|---|
| rb1 November 11 / 16 | Nov 11: 153436, 153721, 155542 | Nov 16: **095242 only** |
| rb2 November 11 | 135934, 142254, 152132 | 135453, 140322, 141227, 142036, 142604, 152844, 154204 |
| rb2 October 30 | Not used for these fits | 184826, 183134, 181003, 181819, 184316: independent floor check |

All dates are in 2026. November 11 uses the shared rb2 map from the example; November 16 uses `20261116-105215-line`, and October uses `20261030-185036-fixed`, both under `data/rb2/racerbot_auto/`.

rb1 `153409` has no moving samples; November 16 `095058` and `095652` have no usable rollout windows. Some evaluation drives had been analyzed in earlier work, so this split is not historically untouched.

## What the real-car measurements showed

The wheel/ground ratio is the median of trusted forward-moving samples above 0.5 m/s. A ratio of 1.075 means the wheels report about 7.5% too much speed.

The gyro measures turning. Its agreement is fitted as **gyro = scale × SLAM yaw rate + bias**; scale near one and bias near zero mean close agreement. RMS (root-mean-square) summarizes the size of the uncorrected gyro–SLAM difference.

| November split | Wheel/ground | Gyro scale | Gyro bias rad/s | Gyro–SLAM RMS rad/s |
|---|---:|---:|---:|---:|
| rb1 development | 1.075 | .994 | +.013 | .181 |
| rb1 evaluation | 1.066 | .968 | +.026 | .162 |
| rb2 development | 1.026 | .983 | +.007 | .156 |
| rb2 evaluation | 1.026 | .963 | +.020 | .201 |

Below is **median absolute sideslip at the centre of gravity**, in degrees, grouped by turning acceleration. We use `a_y = ground speed × yaw rate` as a lateral-acceleration proxy, not a direct force measurement.

| November split | \|a_y\| 1–2 m/s² | 2–3 | 3–4 | ≥4 |
|---|---:|---:|---:|---:|
| rb1 development | 2.1° | 2.8° | 3.1° | 3.4° |
| rb1 evaluation | 2.9° | 3.8° | 4.3° | 6.7° |
| rb2 development | 1.6° | 3.2° | 2.8° | 2.1° |
| rb2 evaluation | 2.6° | 5.0° | 5.3° | 3.7° |

These pooled motion statistics weight samples, not runs equally. rb1's motion summary includes a few sparse evaluation samples; its rollout scores use only `095242`.

## Where the default simulator fell short

These are **pre-speed-change defaults** with default tyres, averaged over eligible windows. Speed, yaw and slip are RMS errors within each window; position is the distance apart at its end.

| Evaluation set | Horizon | Windows | Speed m/s | Yaw rad/s | CG slip ° | Position m |
|---|---:|---:|---:|---:|---:|---:|
| rb1 November | 2 s | 44 | .361 | .335 | 7.969 | .613 |
| rb1 November | 4 s | 24 | .373 | .349 | 7.862 | 1.323 |
| rb2 November | 2 s | 450 | .314 | .315 | 4.997 | .445 |
| rb2 November | 4 s | 307 | .311 | .322 | 5.104 | .950 |
| rb2 October | 2 s | 126 | .245 | .252 | 3.194 | .449 |
| rb2 October | 4 s | 113 | .242 | .244 | 3.165 | .962 |

In November 4-second windows at \|a_y\| ≥3 m/s², the default simulator turns less than measured: turn-aligned yaw bias is **−.249 rad/s rb1**, **−.211 rb2**. Simulated slip magnitude is lower by **4.77°** and **3.72°**. This does not identify a tyre-only cause.

The example command now uses rb2's updated default. To reproduce the old baseline, add `--floor pre_slam_speed_2026_11_rb2` to `slam_rollout.py`.

## Tyre fitting: useful negative evidence

We fitted the existing saturating tyre model separately for each car, inspired by [On-Track SysID](https://arxiv.org/abs/2411.17508). Stiffness controls the response at small slip; peak and shape control how lateral force levels off. Mass, centre of gravity and inertia stayed fixed.

| Car | Front / rear stiffness | Front / rear peak | Shape |
|---|---|---|---:|
| rb1 | 5.153505 / 4.713262 | .398151 / .689094 | 1.537130 |
| rb2 | 4.333613 / 5.882537 | .529682 / .583683 | 1.672428 |

The fit used 2-second development rollouts, equally weighting normalized yaw, CG slip and end-position errors. Their scales were .30 rad/s, .10 rad and .50 m. Adding the external grip clamp or ProgressiveGrip worsened development loss for both cars.

Acceptance required better yaw, slip and position means at **both 2 and 4 seconds**, no worse medians, and speed RMS no more than .02 m/s worse. rb2 also had to pass the October check.

- **rb1 rejected:** its 4-second evaluation position mean improves 1.323→1.276 m, but the median worsens .900→1.156 m.
- **rb2 rejected:** November 4-second yaw RMS .322→.325 rad/s, slip 5.104→5.393°, position .950→1.228 m all worsen.

Neither tyre candidate became default. A plausible model that fails on other drives is evidence to keep, not a reason to relax the rule.

## Speed scaling: rb2 accepted, rb1 rejected

The exact development wheel/ground medians were **rb1 1.0746581411238885** and **rb2 1.0261486728316354**. They were frozen before evaluation; tyres remained at default.

Odometry (the wheels' motion estimate) multiplies true speed by this ratio. Command scaling is separate: **command eRPM gain / absolute odometry gain / wheel ratio**. eRPM is the motor controller's electrical rotation rate. Recorded gains give **1.059513 rb1** (4600/4040) and **1.010890 rb2** (4614/4447.983786). Neither is simply the inverse wheel ratio.

Acceptance required speed RMS **mean and median** to improve at both 2 and 4 seconds; yaw, slip and position means could worsen by no more than 2%. October was a reported independent check, not another gate for this candidate.

| November 4-second evaluation | Speed RMS mean / median, before → after | Signed speed bias, before → after | Decision |
|---|---|---|---|
| rb1 | .373 / .361 → .424 / .415 m/s | +.098 → +.247 m/s | Rejected; default unchanged |
| rb2 | .311 / .289 → .306 / .284 m/s | −.066 → −.042 m/s | Accepted; both scales now default |

Positive bias means the simulator is faster than measured. rb2's October speed mean slightly worsened (.242→.243 m/s at 4 seconds), so the improvement is not universal. The previous rb2 speed defaults remain selectable with the floor name given above.

## What this cannot prove

- **Not independent ground truth:** `loc.py` uses wheel speed and gyro to predict motion before scans correct it. Good map-match quality does not remove that dependence.
- **One rb1 evaluation drive:** there is not enough evidence to claim broad floor coverage.
- **Unmeasured centre of gravity:** localized poses refer to the rear axle; converting to the CG assumes a .18 m rear-axle-to-CG distance. Mass and inertia also remain unmeasured Gym defaults.
- **No rack sensor:** the actual steering angle was not recorded. Initial steering uses the command, not a measured wheel angle.
- **No physical validation:** differentiation amplifies pose noise, and sparse steady driving does not reliably identify axle forces. These are offline comparisons, not permission to skip the physical car's safety checks.

For full manifests, mean/median tables and reproducible fitting commands, continue with the [workspace calibration report](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/d77bd05c0d0339566ba3f5f21202413b5dd14394/docs/sim-calibration.md#measuring-the-real-car-from-slam-localization-november-2026).
