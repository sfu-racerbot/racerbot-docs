# What winning teams put in a lookup table

> **Who this is for:** anyone choosing the next Car 2 pursuit experiment.
>
> **Read first:** [How Racerbot follows a line](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/76db7abecd8c66ca14bbc56d77e82331a08e3cb6/src/pure_pursuit/README.md) for the controller basics.
>
> **You'll be able to:** distinguish three uses of a vehicle model and decide which to test.
>
> **Time:** 4 minutes.

**Yes: a real F1TENTH winner used a steering lookup table.** [Pure pursuit](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/76db7abecd8c66ca14bbc56d77e82331a08e3cb6/docs/glossary.md) steers toward a point ahead on the racing line (the planned path). It assumes the tires follow the commanded arc; at speed they can slip, and the car can run wide.

A lookup table stores answers computed ahead of time. The winning controller could read the steering needed for a turn without solving a tire model on board every time it steered.

## What actually raced

| Team and result | Where the model was used | What the result establishes |
|---|---|---|
| **ForzaETH**, [2022 F1TENTH Germany winner](https://ee.ethz.ch/news-and-events/d-itet-news-channel/2022/08/forzaeth-team-wins-gold.html) | [Becker et al., §§III-C, IV-D](https://arxiv.org/html/2209.04346v4): fit a vehicle and tire model, simulate steady turns offline, then look up steering angle from measured forward speed and requested sideways acceleration. | The authors identify this controller on the winning car and report fastest lap. That does not isolate the table as the cause of the win. |
| **VAUL**, [ICRA 2024 F1TENTH winner](https://www.vaul.fsg.ulaval.ca/en/competitions/) | In a [post-race interview](https://foxglove.dev/blog/spotlight-building-a-championship-autonomous-f1-racecar-for-f1tenth), the team describes mapping the circuit, computing a line and speed profile for its car model, and tracking that line with pure pursuit. | This is a different place to use a model: **where and how fast to drive**, not necessarily **how much to steer**. The interview does not identify a steering table. |

ForzaETH also compared controllers on the **same reference trajectory**, outside competition. At 70% of planned speed, [Table I](https://arxiv.org/html/2209.04346v4#S4.SS3) gives these results:

| Controller | Average lap | Average distance from line |
|---|---:|---:|
| Geometric pure pursuit | 8.74 s | 0.115 m |
| Pursuit with nonlinear tire table | 8.29 s | 0.048 m |

At 80%, geometric pursuit did not complete the five-lap test; the nonlinear version did. These are their car's results, not a predicted Racerbot gain.

## What full-size Formula 1 teams document

- **Mercedes** ([2019 constructors' champion](https://www.formula1.com/en/results/2019/team)) describes [driver-adjustable brake migration](https://www.mercedesamgf1.com/news/how-does-an-f1-steering-wheel-work): front/rear brake balance changes with how hard the driver brakes. This is an input-dependent control setting, not autonomous steering. The team does not disclose its numerical map there.
- **McLaren** ([2024 constructors' champion](https://www.formula1.com/en/results/2024/team)) describes [recreating track conditions in its driver-in-the-loop simulator](https://www.mclaren.com/racing/latest-news/mclarenracing/article/secrets-formula-1-simulator/), including measured wind and its effect on balance. That account describes a development method, not the controller responsible for a particular win.

The transferable F1 practice is to compare the model's predictions with the actual car before trusting a setup change. Neither source says an F1 car follows a racing line with autonomous pure pursuit.

## One Car 2 experiment: change the steering answer, not the target

**Test a speed-indexed steering table against the same adaptive pursuit without it.** Keep [the existing target selection](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/ddaa71a9db715038105ac1666a6614ec835d34ef/src/pure_pursuit/pure_pursuit/tracking_laws.py#L293-L355), racing line, and speed profile fixed. The question is whether Car 2 tracks the line more closely at the same pace, not whether we can command more speed.

Car 2 already has an [optional one-dimensional correction](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/76db7abecd8c66ca14bbc56d77e82331a08e3cb6/src/pure_pursuit/pure_pursuit/racing_math.py#L218-L261): one curve relates requested to delivered sideways acceleration across speeds. It also [feeds the online speed ceiling](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/76db7abecd8c66ca14bbc56d77e82331a08e3cb6/src/pure_pursuit/pure_pursuit/pure_pursuit_node.py#L3525-L3541). Turning it on would not isolate steering. A [previous simulator sweep](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/76db7abecd8c66ca14bbc56d77e82331a08e3cb6/tools/f1tenth_sim/course_suite/results/2026-11-13-plan-sweep.md) reported contacts and cautioned that its steering implementation was not verified against the node.

**1. Qualify the table offline.** Fit steady turns from Car 2's November 11 runs 135934 and 142254, not ETH's tire coefficients. Match steering commands to the delayed turn response. Estimate sideways acceleration from ground speed times yaw rate (how fast the body rotates). A steering command is not a measured wheel angle.

Reserve whole drives, such as 140322 and 141227, before fitting. If separate curves for different speeds do not reduce turn-rate error on those drives compared to one pooled curve, stop here.

**2. Change only the steering calculation.** In simulation, select steering from measured speed and requested sideways acceleration. Use only well-sampled turns where more steering still produces more turn. For sparse cells, stale speed, or requests outside the table, revert to baseline steering. Never extrapolate.

Keep the speed-ceiling curve unset in both arms. Preserve steering and steering-rate limits, braking, the deadman switch and obstacle checks. **Passing stays disabled.**

**3. Run paired laps.** Use the [November 11 saved map and profiled line](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/tree/76db7abecd8c66ca14bbc56d77e82331a08e3cb6/data/rb2/racerbot_auto/20261111-151026-rebuilt). Its grid matches the map published in [run 152132](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/tree/76db7abecd8c66ca14bbc56d77e82331a08e3cb6/data/rb2/racerbot_runs/20261111-152132/bag) cell for cell. That verifies the source map, not every wall in the imported simulator.

Give both versions identical starting poses, floor model and sensor-noise seeds. Repeat on a synthetic course without retuning. Count every contact and non-finish, including failed starts; report first laps separately from laps already at speed.

**4. Decide before chasing lap time.** Require zero contacts and non-finishes, and no loss of minimum body-to-wall clearance. Look for lower 95th-percentile distance from the line without slower median flying laps. Record time spent using the table: a lap driven entirely on fallback cannot demonstrate a gain.

This is a test proposal. No new controller was run, no car settings changed, and there is no Car 2 speed or safety claim.
