# What winning teams put in a lookup table

> **Who this is for:** anyone choosing the next Car 2 pursuit experiment.
>
> **Read first:** [How Racerbot follows a line](https://github.com/sfu-racerbot/Racerbot-Car-Workspace/blob/76db7abecd8c66ca14bbc56d77e82331a08e3cb6/src/pure_pursuit/README.md) for the controller basics.
>
> **You'll be able to:** distinguish three uses of a vehicle model and decide which to test.
>
> **Time:** 4 minutes.

**Yes: a real F1TENTH winner used a steering lookup table.** Pure pursuit steers toward a point ahead on the racing line. A geometric controller assumes the tires follow the commanded arc; at speed they can slip, and the car can run wide. A table can account for this without solving a tire model on the car every time it steers.

## What actually raced

| Team and result | Where the model was used | What the result establishes |
|---|---|---|
| **ForzaETH**, [2022 F1TENTH Germany winner](https://ee.ethz.ch/news-and-events/d-itet-news-channel/2022/08/forzaeth-team-wins-gold.html) | [Becker et al., §§III-C, IV-D](https://arxiv.org/html/2209.04346v4): fit a vehicle and tire model, simulate steady turns offline, then look up steering angle from measured forward speed and requested sideways acceleration. | The authors identify this controller on the winning car and report fastest lap. That does not isolate the table as the cause of the win. |
| **VAUL**, [ICRA 2024 F1TENTH winner](https://www.vaul.fsg.ulaval.ca/en/competitions/) | In a [post-race interview](https://foxglove.dev/blog/spotlight-building-a-championship-autonomous-f1-racecar-for-f1tenth), the team describes mapping the circuit, computing a line and speed profile for its car model, and tracking that line with pure pursuit. | This is a different place to use a model: **where and how fast to drive**, not necessarily **how much to steer**. The interview does not identify a steering table. |

ForzaETH also compared controllers on the **same reference trajectory** outside competition. At 70% speed scaling, Table I gives an average lap of **8.74 s** and average deviation of **0.115 m** for geometric pursuit, versus **8.29 s** and **0.048 m** for pursuit with the nonlinear tire table. At 80%, geometric pursuit crashed after one lap; the nonlinear version completed the comparison. These are their car's results, not a predicted Racerbot gain.

## What full-size Formula 1 teams document

- **Mercedes** ([2019 constructors' champion](https://www.formula1.com/en/results/2019/team)) describes [driver-adjustable brake migration](https://www.mercedesamgf1.com/news/how-does-an-f1-steering-wheel-work): front/rear brake balance changes with how hard the driver brakes. This is an input-dependent control setting, not autonomous steering. The team does not disclose its numerical map there.
- **McLaren** ([2024 constructors' champion](https://www.formula1.com/en/results/2024/team)) describes [recreating track conditions in its driver-in-the-loop simulator](https://www.mclaren.com/racing/latest-news/mclarenracing/article/secrets-formula-1-simulator/), including measured wind and its effect on balance. That account describes a development method, not the controller responsible for a particular win.

The transferable F1 practice is to compare the model's predictions with the actual car before trusting a setup change. Neither source says an F1 car follows a racing line with autonomous pure pursuit.
