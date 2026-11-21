# Does Neural Racer steer like the recorded cars?

> **Who this is for:** club members who want to test a learned driver without moving either car.
> **Read first:** the [car glossary](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/a7b766acfb8901ccc495b407b7f98dfc8e5d8849/docs/glossary.md) if terms like *scan* or *odometry* are new.
> **You'll be able to:** compare the driver's commands across speeds and courses, and tell whether a better score carries over to a simulated race.
> **Time:** 5 minutes to read; allow extra time for data downloads and replay.

Neural Racer was trained in simulation. The car software defaults to **shadow mode**: it predicts commands without driving. The offline evaluator takes a different approach: it shows the driver a recording and compares its answers with the commands the car received.

## Try it on a laptop

No car, ROS installation or graphics card is required for recorded-data replay. Start with the checkout containing the evaluator. This uses a separate folder and does not change any existing car workspace.

**Terminal 1 — laptop:**

```bash
git clone https://github.com/sfu-racerbot/Racerbot-Car-Workspace.git neural-racer-study
cd neural-racer-study
git checkout --detach a7b766acfb8901ccc495b407b7f98dfc8e5d8849
python3 -m venv "$HOME/.venvs/neural-racer-replay"
source "$HOME/.venvs/neural-racer-replay/bin/activate"
python -m pip install -r tools/neural_racer/requirements-real-runs.txt
python tools/neural_racer/real_runs.py --help
```

**Working when:** help lists `extract`, `evaluate` and `simulate`. If you see a missing Python module, check that the virtual environment is active and the install command succeeded.

1. **Extract:** turn the laser scans and wheel speeds from the two cars into examples. Keep a scan only when the left bumper was held, inputs were fresh and a command arrived within 25 milliseconds.
2. **Evaluate:** load [the exported model](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/pull/91) and score its steering and speed against recorded commands. To test a correction, fit a steering multiplier for each speed band on whole days, and compare it on a different day.
3. **Simulate:** run both versions with the same course, start position, speed cap and seed (the simulator's random starting value). Try starting alone and behind the existing gap-follow driver. Compare finishes, contacts and distance, not just steering error.

The [workspace guide](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/a7b766acfb8901ccc495b407b7f98dfc8e5d8849/docs/neural-racer-real-runs.md) gives the commands, expected outputs and dataset fields. Nothing is installed on a car and no model is approved for driving.

## How to read the first result

Of 36 recordings, 35 yielded usable examples. We fitted on October 19 and 26 and replayed 34,146 examples from October 30. A smaller steering error means the output was closer to the recorded command, **not** that it would have chosen a better route.

| October 30 measurement | Original driver | With speed-based correction |
|---|---:|---:|
| Average steering error, all examples | 0.0930 rad | 0.0893 rad |
| At speeds from 0 to below 1 m/s | 0.1125 rad | 0.1159 rad (worse) |
| At speeds of at least 3 m/s | 0.0662 rad | 0.0518 rad |
| In the two recordings with both cars on track | 0.0915 rad | 0.0881 rad |
| Average speed command error | 2.207 m/s | 2.207 m/s |

Replay is like marking answers on a fixed route: a different steering choice cannot change the next recorded scan. Simulation lets that choice change the car's path. **Neither version finished a lap in 12 simulator trials each.** October 30 helped choose this correction, so a fresh day is still needed for an independent test.

The [two-car labels](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/a7b766acfb8901ccc495b407b7f98dfc8e5d8849/docs/rb1-lap-gap-2026-10-30.md#the-runs) mean both cars were on the course during a session. They do not say an opponent was in view on each scan. Where the course or traffic is not established, the scorecard reports `unknown` instead of guessing.

**Keep the correction offline.** The recordings lack the motion sensor's health messages, so replay marks that sensor absent. Never disable the live health check or the LB deadman (the bumper the operator must hold to allow motion) to reproduce a replay score.
