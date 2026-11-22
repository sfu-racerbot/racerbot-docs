# Compare Car 2's cornering on the November 11 race floor

> **Who this is for:** someone testing a controller or a learned policy in the club simulator.
>
> **Read first:** [club simulator setup](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/76db7abecd8c66ca14bbc56d77e82331a08e3cb6/tools/f1tenth_sim/README.md) — use the club's Python simulator, not the ROS bridge container.
>
> **You'll be able to:** replay the same simulated lap with two grip settings, without changing any defaults.
>
> **Time:** a few minutes with the simulator installed.

**Use 1.15 for the race-lap baseline. Try 1.25 as a second test when the question is how quickly the car turns.** These numbers scale this floor's measured cornering curve; they are not tyre friction coefficients. A higher number need not give a more accurate lap.

## Run the comparison

**Terminal — your development computer**, from the root of [the pinned club workspace](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/tree/76db7abecd8c66ca14bbc56d77e82331a08e3cb6), after simulator setup. This runs only Python and writes two summaries to a new temporary directory. Do not start ROS or connect to a car.

```bash
(
set -e
export PYTHONPATH="$PWD/.sim/python${PYTHONPATH:+:$PYTHONPATH}"
out=$(mktemp -d)
for grip in 1.15 1.25; do
  RACERBOT_CAR=rb2 python3 tools/f1tenth_sim/start_replay.py \
    --track car2_course_2026_11_11 --floor sfu_course_2026_11_11_rb2 \
    --line data/rb2/racerbot_auto/20261111-151026-rebuilt/raceline_profiled.csv \
    --start-s 8.1 --start-speed 3.42 --start-right 0.098 --start-heading-error 0.48 \
    --online-lateral 3.1 --law delay_lqr --seeds 1 --laps 4 --seconds 70 \
    --grip-scale "$grip" --out "$out/grip-$grip.json"
done
echo "$out"
)
```

**Working when:** each run prints a seed, contact count and lap times. For seed 12345 the first complete flying lap (already at speed) took about 10.08 s at 1.15 and 9.80 s at 1.25. These are simulator laps, not measurements from a new drive.

If Python cannot find Gym, finish the linked setup. If it cannot find the line file, check the workspace revision and directory. Omitting `--grip-scale` uses the floor's [configured 1.15](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/76db7abecd8c66ca14bbc56d77e82331a08e3cb6/tools/f1tenth_sim/sim_fidelity/calibration.py#L592-L601); the help text at this revision incorrectly says 1.25.

## Why test both?

In the **turn-rate test**, the simulator received the car's recorded commands instead of steering itself. Its body rotation was compared with the gyroscope. Fit runs were November 11 **135934, 142254 and 152132**. Seven other complete drives were reserved before selecting 1.25.

| Reserved drive (20261111-) | Baseline 1.15 | Optional 1.25 |
|---|---:|---:|
| 135453 | 0.235 | 0.216 |
| 140322 | 0.331 | 0.292 |
| 141227 | 0.236 | 0.214 |
| 142036 | 0.312 | 0.262 |
| 142604 | 0.360 | 0.322 |
| 152844 | 0.455 | 0.411 |
| 154204 | 0.335 | 0.299 |
| **Mean, each drive weighted equally** | **0.324** | **0.288** |

The table shows root mean square error in turn rate (rad/s; lower is better). The optional setting reduced it on every drive. A separate estimate from matching laser scans also improved, from 0.304 to 0.291 rad/s, but scan matching is not an external tracking system.

In the **lap test** above, the controller steers itself. The [recorded map-racer reference](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/76db7abecd8c66ca14bbc56d77e82331a08e3cb6/tools/run_analysis/results/2026-11-14-rb2-race-day-agreement.md) was 10.05 s. That drive helped select the floor; it is not an independent lap test. Greater grip can improve turn-rate tracking while predicting a lap that is too fast.

<details>
<summary>What counted as a corner, and what the recordings cannot tell us</summary>

Replay the final command sent by the command selector (`/ackermann_cmd`) every 25 ms on the bag's receive clock. Start each 10-second score block from rest with two preceding seconds of command history. Compare the simulated state at the end of the tick with the calibrated, car-axis VESC gyro at that time.

Score only forward commands with wheel speed above 1 m/s and absolute steering command at least 0.10 rad. Discard a tick if a command went stale (over 150 ms old) in its last two seconds. Also discard it if wheel or gyro readings are more than 100 ms apart. The seven drives contributed 442.5 seconds of corners. Only 5.15 seconds came from 152844.

The signal called `/steering/measured` infers an angle from rotation and speed. The servo feedback repeats its command. Neither measures the angle of the road wheels under load. Wheel speed is not ground speed when tyres slip. Do not use these estimates to certify a policy for the real car.

</details>
