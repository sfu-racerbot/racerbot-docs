# Check where Car 2's simulated laser starts

> **Who this is for:** anyone comparing simulated laser distances with Car 2.
> **Read first:** [Car 2's mount measurements][dimensions].
> **You'll be able to:** check the simulated mount against a wall of known position.

Place the rear axle 2.00 m from a wall and point the car straight at it.
The real mount puts the laser 0.26 m closer to the wall, so its forward beam
should read **1.74 m**. In the default simulator the beam instead reads
about **1.56 m**. Both numbers are distances from the scanner, not the bumper.

## Why two offsets describe one mount

The tape measurements put the axles 0.36 m apart and the scanner 0.10 m behind
the front axle. The [recorded transform][transform] (the position of one named
frame relative to another) places the scanner 0.26 m ahead of the rear axle
and 0.11 m above it, with no rotation. A frame is the point and directions
used as the reference for a measurement.

The [single-track simulator][gym] tracks the modeled center of gravity, not
the rear axle. By default that point is halfway between the axles, but the
midpoint is a modeling assumption, not a measured weight distribution.

| Distance along the car | Default simulator | Measured-mount option |
|---|---:|---:|
| Rear axle to modeled center | 0.18 m | 0.18 m |
| Modeled center to start of ray | 0.26 m | 0.08 m |
| Rear axle to start of ray | 0.44 m | 0.26 m |

## Check the wall distance without driving

In a terminal on a development machine, start in the main folder of the
Car Workspace repository **with the companion simulator patch applied**.
Use its [pinned Gym setup][setup] and a
Python environment with pytest installed. No robot or ROS graph is needed.

```bash
bash tools/f1tenth_sim/setup.sh
export PYTHONPATH="$PWD/.sim/python:$PWD/.sim/f1tenth_gym:${PYTHONPATH:-}"
python3 -m pytest -q tools/f1tenth_sim/tests/test_measured_lidar_mount.py
```

The tests should pass. They check actual beams facing east and north, in both
the Levine evaluator and the simulator bridge. They also move the modeled
center: the scanner must stay fixed relative to the **rear axle**.

## Select the frame for an offline run

Add `--car rb2 --measured-lidar-mount` to a single-car `eval_gap.py` run.
Omit the mount flag to reproduce the default. For the ROS simulator bridge,
the corresponding launch arguments are `car:=rb2 measured_lidar_mount:=true`
on `sim_bringup_launch.py`. Neither option changes the car's physical settings.

With the option on, the bridge starts its odometry (estimated position) at
the rear axle; it can still drift as before. The physical `base_link` (rear axle) to `laser` transform
stays unchanged. Simulator truth and reset poses still refer to the modeled center.

**This is not a controller safety fix.** In a matched Levine run, the default
timed out without contact. With the measured mount it completed one lap, then
hit a wall at 19.42 s. Do not infer a safe speed or a new stop threshold from
this geometry check. It does not validate another car or model reflected returns.

[dimensions]: https://github.com/sfu-racerbot/Racerbot-Car-2-Workspace/blob/9c2c2ed7bbf675674f30ca5937432a62b0eab3a3/docs/hardware-reference.md#L127-L137

[transform]: https://github.com/sfu-racerbot/Racerbot-Car-2-Workspace/blob/9c2c2ed7bbf675674f30ca5937432a62b0eab3a3/data/racerbot_runs/20261030-185036/metadata/params/static_baselink_to_laser.yaml

[gym]: https://github.com/f1tenth/f1tenth_gym/blob/bdaec1420c3b0f103858d289866d0d4e2e597c30/f1tenth_gym/envs/simulator.py#L361-L394

[setup]: https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/7b2003f5db5b627e4326ca3423902dce5201778d/tools/f1tenth_sim/setup.sh
