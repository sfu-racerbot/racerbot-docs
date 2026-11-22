# When a better turn prediction makes a worse race lap

> **Who this is for:** club members comparing racing algorithms on the November Car 2 map.
> **Read first:** [compare two cornering settings](CAR2_NOVEMBER_CORNERING.md) for the simulator setup.
> **You'll be able to:** compare the original car and an experimental car without changing either default.
> **Time:** five minutes with Gym installed.

**Use the original model for racing.** The experimental car matched recorded steering commands more closely, but when the controller chose its own commands, it finished laps too quickly. Do not use the faster simulated time to raise speed limits on a real car.

| Car 2 comparison | Original | Experiment | What the recording tells us |
|---|---:|---:|---|
| Turning-rate error across seven reserved drives | 0.313 rad/s | 0.175 rad/s | Smaller is better; measured by a gyroscope |
| Position error after four seconds | 1.18 m | 0.96 m | Smaller is better; estimated from laser scans against a map |
| Mean lap time over six simulator seeds | 10.05 s | 9.33 s | Recorded lap: 10.05 s; this lap helped calibrate the original |

Car 1 also has a saved experiment, not a new default. Its turning-rate error improved only 4%, and its position got worse on the only evaluation drive with reliable mapped windows.

## Try one paired lap comparison

**Terminal — development computer.** Use the [merged workspace revision](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/tree/3ad77c8a0f12a1f0c6b35440784663a0bb0cf8d9) with the simulator installed. Run from its root, without ROS or a connection to either car.

```bash
(
set -e
export RACERBOT_CAR=rb2
export PYTHONPATH="$PWD/.sim/python:$PWD/tools:$PWD/src/drive_intent${PYTHONPATH:+:$PYTHONPATH}"
out=$(mktemp -d)
for mode in original experiment; do
  args=()
  if [[ $mode == experiment ]]; then
    args=(--turn-response tools/f1tenth_sim/turn_response_study/rb2.json)
  fi
  python3 tools/f1tenth_sim/start_replay.py \
    --track car2_course_2026_11_11 --floor sfu_course_2026_11_11_rb2 \
    --line data/rb2/racerbot_auto/20261111-151026-rebuilt/raceline_profiled.csv \
    --start-s 8.1 --start-speed 3.42 --start-right 0.098 --start-heading-error 0.48 \
    --online-lateral 3.1 --law delay_lqr --seeds 1 --laps 4 --seconds 70 \
    "${args[@]}" --out "$out/$mode.json"
done
)
```

**Working when:** each run prints seed 12345 and zero contacts. Its first lap already at speed takes about 10.08 s with the original and 9.30 s with the experiment. Omitting the experimental flag always leaves this comparison on the original floor model.

**If it doesn't:** an unknown flag means your workspace lacks the experiment. Use the linked revision. A missing line or map means the workspace's November course data is unavailable; do not substitute a different course and compare its times to this table.

See the [maintainer handoff](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/3ad77c8a0f12a1f0c6b35440784663a0bb0cf8d9/docs/ai-handoff/turn-response.md) for the independent turning-rate check, the map-based estimates, what cannot be inferred from wheel speed, and the next measurement needed.
