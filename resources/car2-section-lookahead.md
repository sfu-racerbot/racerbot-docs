# Should Car 2 look farther ahead on parts of the track?

> **Who this is for:** Club members deciding which steering changes deserve another simulator test.
>
> **Read first:** [Racing controller guide](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/feature/adaptive-pursuit-racing/docs/racing-autonomy.md).
>
> **You'll be able to:** Explain why a faster lap on one map did not justify changing the default.
>
> **Time:** 3 minutes.

**Keep current adaptive pursuit.** It steers toward a point ahead on the car's planned path (the [racing line](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/feature/adaptive-pursuit-racing/docs/glossary.md)). We changed only how far ahead it looks on six sections of each map. Only November 16 got faster without losing wall clearance.

This was a solo simulator test, not a drive on Car 2. Passing stayed off. We did not change the speed profile, hardware settings, obstacle checks or stopping limits. Wait for the revised tyre model before trying more settings.

## How the laps compared

A flying lap starts after the car is already moving. Each time below is the median of the completed flying laps. Wall clearance is the smallest distance from the eight sampled outline points to the simulator walls across all three runs, including failures.

| Course | Flying lap: current → sections (s) | Smallest wall clearance (cm) | Wall contacts: current → sections |
|---|---:|---:|---:|
| November 9 | 13.927 → 14.051 | 0 → 0 | 1 → 1 |
| November 11 | 10.074 → 10.120 | 10.0 → 10.0 | 0 → 0 |
| November 16 | 8.961 → 8.743 | 5.0 → 7.1 | 0 → 0 |
| Synthetic wide oval | 13.107 → 13.212 | 25.0 → 22.4 | 0 → 0 |

November 9 lost different seeds in the two arms, so its surviving laps are not a like-for-like speed comparison. On November 16 all six paired flying laps improved, but that course has no paired recorded grid. The simulator's ground model came from November 11, not November 16.

### What did the car hit on November 9?

All four contacts (two during fitting, two during judging) hit the same isolated 5 cm square that an earlier replay found near waypoint 231. The simulator treats it as a wall. Car 2's recorded map labels that square and all eight neighbors free. A free label does not prove the real room was empty.

The saved map allows occupancy probability below 0.25 as free. This square was 0.196; the simulator importer requires less than 0.10. Its closest non-free recorded square was 0.985 m away, measured between cell centers. The recorded grid matched the saved map across all 138,180 cells.

The judged current-pursuit run began avoidance at a reported 1.42 m. It issued a zero-speed command at 10.575 s but was still moving at 2.23 m/s at 10.650 s. At impact the closest sampled outline point was 0.608 m from a recorded occupied cell; the section-based run's margin at impact was 0.632 m.

A replay that only watched the car reproduced all four original position histories exactly. No replay touched a recorded occupied cell. Keep the imported cell as a stress test, not as evidence of a physical crash or a reason to weaken obstacle checks.

<details>
<summary>Study design, frozen settings and every judged run (optional)</summary>

The [experiment code](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/tree/8f546313a836c32e72afafbd15489c000f03f5e4/tools/f1tenth_sim) adds a small distance to the existing adaptive target, blends between sections and clips to the original limits. It does not replace steering geometry.

Each course had six equal-length sections. We tried one uniform offset at a time: 0, −10 or +10 cm. The two training seeds were 4101 and 4102. For each section we compared the second circuit's time and the worst clearance across all visits in each seed.

Failed or contacting candidates and candidates losing section clearance were excluded. If the baseline did not finish but did cross a section, its observed clearance was still the floor. We froze the fastest eligible choice per section before running the three fresh judging seeds.

| Map | Offsets in driving order (cm) |
|---|---|
| November 9 | −10, −10, −10, −10, −10, −10 |
| November 11 | +10, 0, −10, −10, 0, −10 |
| November 16 | 0, 0, 0, +10, −10, −10 |
| Synthetic wide oval | +10, +10, −10, +10, 0, −10 |

Both arms used each course's same start, speed profile, imported grid and sensor-noise seed. New seeds are not new courses or independent physical drives. The November 11 recording ending in 152132 informed the simulation; it was not a held-out drive.

“First” measures from rest to the last waypoint before the start line; subsequent crossings at that gate give full flying laps. A dash means no completed lap. Line error is the 95th percentile of simulated position distance to the line. All runs started; neither arm had another car to hit.

| Course | Seed | Arm | First (s) | Flying laps (s) | Wall contacts | Min wall (cm) | Line error (cm) | Finished? |
|---|---:|---|---:|---:|---:|---:|---:|---|
| nov09 | 5101 | adaptive | — | — | 1 | 0.0 | 32.5 | No |
| nov09 | 5101 | sector | 14.565 | 14.122 / 13.957 | 0 | 5.0 | 30.9 | Yes |
| nov09 | 5102 | adaptive | 14.246 | 13.794 / 13.921 | 0 | 5.0 | 34.5 | Yes |
| nov09 | 5102 | sector | 14.571 | — | 1 | 0.0 | 31.6 | No |
| nov09 | 5103 | adaptive | 14.255 | 13.989 / 13.933 | 0 | 5.0 | 35.3 | Yes |
| nov09 | 5103 | sector | 14.316 | 13.997 / 14.104 | 0 | 7.1 | 30.7 | Yes |
| nov11 | 5101 | adaptive | 10.766 | 10.073 / 10.110 | 0 | 10.0 | 25.4 | Yes |
| nov11 | 5101 | sector | 10.806 | 10.105 / 10.159 | 0 | 11.2 | 24.8 | Yes |
| nov11 | 5102 | adaptive | 10.799 | 10.079 / 10.050 | 0 | 10.0 | 24.9 | Yes |
| nov11 | 5102 | sector | 10.839 | 10.145 / 10.122 | 0 | 10.0 | 24.9 | Yes |
| nov11 | 5103 | adaptive | 10.764 | 10.075 / 10.049 | 0 | 10.0 | 25.0 | Yes |
| nov11 | 5103 | sector | 10.801 | 10.117 / 10.088 | 0 | 10.0 | 24.7 | Yes |
| nov16 | 5101 | adaptive | 9.463 | 9.074 / 8.966 | 0 | 5.0 | 29.3 | Yes |
| nov16 | 5101 | sector | 9.282 | 8.779 / 8.704 | 0 | 10.0 | 28.4 | Yes |
| nov16 | 5102 | adaptive | 9.562 | 9.188 / 8.913 | 0 | 5.0 | 29.1 | Yes |
| nov16 | 5102 | sector | 9.304 | 8.776 / 8.676 | 0 | 7.1 | 28.3 | Yes |
| nov16 | 5103 | adaptive | 9.623 | 8.911 / 8.956 | 0 | 5.0 | 28.4 | Yes |
| nov16 | 5103 | sector | 9.523 | 8.710 / 8.832 | 0 | 7.1 | 29.8 | Yes |
| indoor_wide | 5101 | adaptive | 13.783 | 13.054 / 13.202 | 0 | 29.2 | 74.9 | Yes |
| indoor_wide | 5101 | sector | 13.863 | 13.301 / 13.204 | 0 | 22.4 | 77.8 | Yes |
| indoor_wide | 5102 | adaptive | 13.827 | 13.089 / 13.125 | 0 | 25.0 | 77.2 | Yes |
| indoor_wide | 5102 | sector | 13.837 | 13.132 / 13.304 | 0 | 26.9 | 77.8 | Yes |
| indoor_wide | 5103 | adaptive | 13.914 | 13.074 / 13.124 | 0 | 25.0 | 76.0 | Yes |
| indoor_wide | 5103 | sector | 13.921 | 13.157 / 13.220 | 0 | 22.4 | 79.9 | Yes |

Inputs: [Car 2 map packages and recordings](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/tree/76db7abecd8c66ca14bbc56d77e82331a08e3cb6/data/rb2). The map packages were 185146-rebuilt (Nov 9), 151026-rebuilt (Nov 11) and 105215-line (Nov 16); each supplied its own saved line.

Simulator: [F1TENTH Gym revision](https://github.com/f1tenth/f1tenth_gym/tree/bdaec1420c3b0f103858d289866d0d4e2e597c30), the unchanged November 11 Car 2 floor profile, 40 updates per second. The [per-waypoint research](https://arxiv.org/abs/2111.08873) motivated the question; this coarser six-section trial is not a replication.

</details>
