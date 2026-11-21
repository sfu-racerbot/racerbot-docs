# Gap-follow time trials: lap times and safety checks

> **Who this is for:** anyone comparing how the club's cars steer on an empty course.
> **Read first:** [Gap-follow basics](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/main/docs/gap-follow.md) for how the car uses a laser scan to choose a gap.
> **You'll be able to:** repeat a simulated race and check whether a faster lap cost wall clearance.

## What changes on a long straight?

Imagine coming out of a bend. Several laser beams can see all the way down the next straight. Aiming at the beam nearest straight ahead can leave the car pointing at the edge of the opening. Aiming farther inside can line the car up earlier without raising its speed limit.

When a selected gap reaches the scan's 10 m cap, the controller starts with the old target: the nearly deepest beam closest to straight ahead. It finds the uninterrupted band around that beam (returns within 5% of the deepest).

The car aims at the middle of that band only if it is wide enough and both edges are visible. It never takes the middle of two openings separated by a wall.

The setting is `open_aim_min_width`, measured along the arc at the far end of the scan, **not** the width beside the car. Normal driving keeps zero (the old aim); `time_trial:=true` uses 1.24 m for both cars. The time trial is for **one car alone**: it turns off opponent awareness.

**Simulation only.** A fast lap is not evidence that a physical car is ready to race. Keep the LB hold-to-drive control enabled. Check with wheels off the ground before low-speed floor tests; see the [workspace safety checklist](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/main/docs/writing-your-own-node.md). The change does not raise the speed or braking limits.

## How much faster?

On Spielberg, each car finished about a second sooner per lap without passing closer to a wall. These are mean times over six seeds, three laps each, including the start from rest.

| Spielberg | Before (s/lap) | After (s/lap) | Smallest wall clearance, before → after |
|---|---:|---:|---:|
| Car 1 | 72.614 | 71.508 | 8.2 → 13.0 cm |
| Car 2 | 74.068 | 72.850 | 13.0 → 13.0 cm |

The indoor oval, narrow loop, ASB hallway and race-day course had unchanged lap times. All 120 benchmark runs finished without contact.

A separate stress test varied the floor and grip across 12 recorded maps. Both versions had 159 contact runs and 39 stalls out of 432; the new version added none on individual seeds. A narrower 0.62 m setting was faster in the ASB hallway but introduced a wall contact on car 1, so it was rejected.

[Full measurements and test output](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/691b427f4316d0e64490cca1ba9d4c3fd476b8de/tools/run_analysis/results/2026-11-21-open-aim.md) include every lap, wall clearances, the floor comparison and the existing expected test failure.

## Repeat the comparison without driving a car

**One terminal, on a development computer:** start in the [tested car workspace version](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/tree/691b427f4316d0e64490cca1ba9d4c3fd476b8de). The first command installs the pinned simulator. These are headless tests: no controller, ROS launch, or physical car is needed.

```bash
bash tools/f1tenth_sim/setup.sh
PYTHONPATH=.sim/python python3 tools/f1tenth_sim/time_trial_benchmark.py \
  --arm before:open_aim_min_width=0 --arm after: \
  --output .sim/time-trial.json
```

**Working when:** every output line lists three lap times and zero contacts, and the command exits successfully. The JSON keeps every run, including failures. The first lap starts from rest. Compare the same course, car, and seed (the number that repeats the same simulated noise) in each group.
