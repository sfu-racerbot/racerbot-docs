# Car 2: testing stop timing on the October 30 floor

**Who this is for:** anyone testing how long car 2 takes to slow down.

**Read first:** the [simulator setup](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/782ba8ca457e188035e69a338728ae1e6e902189/docs/simulator.md#one-time-setup) in the car workspace (the repository with the simulator).

**You'll be able to:** choose a floor profile and check its stop time against recorded runs.

For **timing after a zero-speed command**, select `--car rb2 --floor sfu_course_2026_10_30_stops`. This floor profile is a set of simulator settings, not a change to the real car. It models the gentler end of a forward stop; ordinary nonzero speed commands work as before.

## What improved?

In the [October 30 recordings](https://github.com/sfu-racerbot/Racerbot-Car-2-Workspace/tree/9c2c2ed7bbf675674f30ca5937432a62b0eab3a3/data/racerbot_runs), the wheels kept turning longer than the original simulator predicted. Eight stops were used to fit the slower response. The other stops below were used only to check it.

| Stop-time error | Original floor | Stop-timing floor |
|---|---:|---:|
| Six held-out stops | 0.242 s | 0.056 s |
| Four further stops outside the original split | 0.263 s | 0.050 s |

The numbers are root mean square error: smaller means the simulation is closer to the recordings. Time is measured from the command to a wheel speed of 0.3 m/s, not to absolute rest.

**For distance to a wall, keep the original floor** (`sfu_course_2026_10_30`). On a stop measured at 3.39 m, it predicts 3.38 m; the stop-timing floor predicts 3.51 m. Better timing does not imply better clearance.

## Repeat the comparison

Use a [workspace revision containing the stop model](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/pull/93). From a terminal at the workspace root, run:

```bash
python3 tools/run_analysis/validate_stop_tail.py
```

Look for `held-out: 6 stops` and `additional: 4 stops`. The command uses included measurements; it does not connect to a car. The [calibration notes](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/782ba8ca457e188035e69a338728ae1e6e902189/docs/sim-calibration.md#car-2-testing-how-long-a-stop-takes) explain how to reproduce the measurements from the original recordings.
