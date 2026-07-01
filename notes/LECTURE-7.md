# Lecture 7 Notes

**Lecture 7:** https://youtu.be/kQi5IGzvr0c?si=89jK2Q1ja-OfM-Ym

## Localization and Mapping: Introduction to Bayes Filter

Lesson plan:
1. Introduction to State Estimation
2. Recap of Probability and Bayes Rule
3. Recursive Bayes Filter
4. Variants of Bayes Filter: KF, Particle Filter
5. Running Particle Filter in ROS2

## State Estimation

Recall the system from vehicle dynamics:

$$
\dot{x} = Ax + Bu, \quad y = Cx
$$

- **System Dynamic** (input → system): how control inputs $u$ (e.g. motor) drive the state
- **Sensor Model** (system → output): how the state shows up as measurements $y$
  - Sensors: IMU (on VESC), LiDAR, GNSS (GPS), camera, ...

State Estimation runs this pipeline *backwards*: use the control inputs and sensor outputs to estimate the internal state (e.g. the car's location).

![State Estimation Pipeline](/assets/module-b/lecture-7/state-estimation-pipeline.png)

### Dead Reckoning

**Definition:** Calculate the current position of the car from a previous position, plus estimates of speed, heading (direction), and elapsed time.

**Does it work?**
- If we had perfectly accurate control inputs and a perfect kinematic model → it would be fine
- But there is always uncertainty in the measurement and system model, which creates cumulative errors

Example: recall the VESC parameters tuned for the car. Can `ros2 topic echo /odom` give an accurate speed/position?

```
# erpm = speed_to_erpm_gain * speed + speed_to_erpm_offset
speed_to_erpm_gain: 4614.0
speed_to_erpm_offset: 0.0
# servo = steering_angle_to_servo_gain * steering_angle + steering_angle_to_servo_offset
steering_angle_to_servo_gain: -1.2135
steering_angle_to_servo_offset: 0.5304
```

These are imperfect estimates → we always have noise.

Key reframe: instead of asking for one exact position, frame state estimation as approximating a *distribution* over the position $p(x)$.

![Dead Reckoning Noise](/assets/module-b/lecture-7/dead-reckoning-noise.png)

### State Estimation as a Distribution

We approximate a distribution using two information sources:
- **Control Input** → **Prediction** (push the belief forward using motion)
- **Observation** → **Correction** (pull the belief toward what the sensors see)

## Recap of Probability and Bayes Rule

### Conditional Probability
- $P(B \mid A)$: the chance of event $B$ when event $A$ has already happened ("probability of $B$ given $A$")

### Bayes Rule
Starting from the two ways to write a joint probability:

$$
P(AB) = P(A)\,P(B \mid A) \quad (1)
$$
$$
P(AB) = P(B)\,P(A \mid B) \quad (2)
$$

Combining gives Bayes Rule:

$$
P(B \mid A) = \frac{P(AB)}{P(A)} = \frac{P(A \mid B)\,P(B)}{P(A)}
$$

In words:

$$
\text{posterior} = \frac{\text{likelihood} \times \text{prior}}{\text{evidence}}
$$

With $A$: evidence (observation), $B$: hypothesis (state):
- **Prior** $P(B)$: the probability distribution (belief) *before* the evidence is considered
- **Likelihood** $P(A \mid B)$: probability of the evidence given the belief → how much can we trust the belief?
- **Posterior** $P(B \mid A)$: updated belief *after* the evidence is considered
- **Evidence** $P(A)$: usually a normalization term so the posterior is a valid PDF

![Bayes Rule](/assets/module-b/lecture-7/bayes-rule.png)

### Law of Total Probability
Decompose a problem by conditioning on another variable.

- **Discrete case:** for mutually exclusive, exhaustive events $B_1, \dots, B_k$:

$$
P(A) = \sum_{i=1}^{k} P(A \cap B_i) = \sum_{i=1}^{k} P(A \mid B_i)\,P(B_i)
$$

- **Continuous case:**

$$
P(A) = \int_{-\infty}^{\infty} P(A \mid X = x)\,f_X(x)\,dx
$$

where $f_X(x)$ is the probability density function (PDF) and $F(x)$ the cumulative distribution function (CDF).

![Law of Total Probability](/assets/module-b/lecture-7/law-of-total-probability.png)

### More Evidence → More Conditions
Bayes Rule extends when we condition on extra context $C_1, \dots, C_n$:

$$
P(B \mid A, C_1, \dots, C_n) = \frac{P(A \mid B, C_1, \dots, C_n)\,P(B \mid C_1, \dots, C_n)}{P(A \mid C_1, \dots, C_n)}
$$

- Combining historical information → Recursive Bayes Filter
- Combining multiple sensor measurements → Sensor Fusion

The belief (posterior) over the robot state conditions on all past observations and controls:

$$
\text{Bel}(x_t) = P(x_t \mid o_t, u_t, o_{t-1}, u_{t-1}, \dots)
$$

- $x_t$: robot state
- $o_t$: current observation, $u_t$: control input, $o_{t-1}, \dots$: history of observations

## Recursive Bayes Filter

### Hidden Markov Model (HMM)

**Goal:** Take the belief at time $t-1$ and advance the estimate of $x$ to time $t$.

Two models drive the chain:
- **Action / Motion model (transition):** $p(x_t \mid u_t, x_{t-1})$: likelihood of the next state, given current control and previous state
- **Sensor model:** $p(o_t \mid x_t)$: likelihood of the current observation, given the current state

### Markov Property (conditional independence)
- The current observation depends only on the current state:

$$
P(O_t \mid x_1, \dots, x_t, u_1, \dots, u_t) = P(O_t \mid x_t)
$$

- The current state depends only on the previous state:

$$
P(x_t \mid x_1, \dots, x_{t-1}, u_1, \dots, u_{t-1}) = P(x_t \mid x_{t-1})
$$

This is what collapses the full history into a simple recursion.

![Hidden Markov Model](/assets/module-b/lecture-7/hidden-markov-model.png)

### Step 1: Prediction (with the control input)
Like dead reckoning, but probabilistic. Start from the predicted belief $\overline{bel}(x_t)$ and apply the Law of Total Probability (condition on $x_{t-1}$):

$$
\overline{bel}(x_t) = P(x_t \mid o_{1:t-1}, u_{1:t})
$$

$$
= \int P(x_t \mid x_{t-1}, o_{1:t-1}, u_{1:t})\,\underbrace{P(x_{t-1} \mid o_{1:t-1}, u_{1:t})}_{\text{recursive term } = \, bel(x_{t-1})}\,d(x_{t-1})
$$

Apply the Markov property to simplify the first term to the action model:

$$
\overline{bel}(x_t) = \int \underbrace{P(x_t \mid x_{t-1}, u_t)}_{\text{action model}}\,\underbrace{bel(x_{t-1})}_{\text{posterior of } x_{t-1}}\,d(x_{t-1})
$$

### Step 2: Correction (with the observation)
Fold in the new observation $o_t$ using Bayes Rule:

$$
bel(x_t) = P(x_t \mid o_{1:t-1}, o_t, u_{1:t})
$$

$$
= \frac{P(o_t \mid o_{1:t-1}, x_t, u_{1:t})\,P(x_t \mid o_{1:t-1}, u_{1:t})}{P(o_t \mid o_{1:t-1}, u_{1:t})}
$$

Apply the Markov property to the likelihood ($o_t$ depends only on $x_t$):

$$
bel(x_t) = \frac{\overbrace{P(o_t \mid x_t)}^{\text{sensor model}}\;\overbrace{P(x_t \mid o_{1:t-1}, u_{1:t})}^{\text{prior } = \, \overline{bel}(x_t)}}{\underbrace{P(o_t \mid o_{1:t-1}, u_{1:t})}_{\text{normalization term}}}
$$

### Practical Issue
Both steps require multiplication and even integration of two probability distributions, generally intractable in closed form. Writing the normalizer as $\eta$:

$$
\overline{bel}(x_t) = \int P(x_t \mid x_{t-1}, u_t)\,bel(x_{t-1})\,d(x_{t-1})
$$
$$
bel(x_t) = \frac{P(o_t \mid x_t)\,\overline{bel}(x_t)}{P(o_t \mid o_{1:t-1}, u_{1:t})} = \eta\,P(o_t \mid x_t)\,\overline{bel}(x_t)
$$

This is why we use variants of the Bayes filter that make the math tractable.

![Bayes Filter Derivation](/assets/module-b/lecture-7/bayes-filter-derivation.png)

## Variants of Bayes Filter

### Assume a Simple (Gaussian) Distribution
- **Kalman Filter (KF)**, **Extended Kalman Filter (EKF)**, **Unscented Kalman Filter (UKF)**

Why Gaussians work so well:
- **Conditional distribution:** if two sets of variables are jointly Gaussian, the conditional of one given the other is again Gaussian
- **Self-conjugate:** a Gaussian likelihood with a Gaussian prior yields a Gaussian posterior

→ The distribution keeps the same form throughout propagation, giving an analytical solution.

### Use a Sampling-Based Method
- **Particle Filter (PF)**: represents complicated, non-Gaussian distributions with samples (covered below)

### KF Example
- **State:** position of the car
- **Observation:** sensor measurement to a pole
- **Assumption:** both state and observation are Gaussian

Walkthrough:
- **[Fig1]** Initial knowledge at $T=0$ (known initial velocity)
- **[Fig2]** Prediction at $T=1$: the action model adds uncertainty → the position Gaussian gets wider (confidence decreases)
- **[Fig3]** A noisy measurement at $T=1$ (its own Gaussian)
- **[Fig4]** Multiply the prediction and measurement PDFs → fused estimate that is sharper than either alone

![KF Gaussian Fusion](/assets/module-b/lecture-7/kf-gaussian-fusion.png)

### EKF Example
On a GNSS-tracked trajectory:
- Blue = true trajectory, Black = dead reckoning, Green = GNSS observations
- Red line = EKF estimate, Red ellipse = EKF covariance estimate

![EKF Trajectory](/assets/module-b/lecture-7/ekf-trajectory.png)

### Practical Problem of KF: Non-Gaussian Noise
When the noise is *not* Gaussian (e.g. GNSS in practice), the KF/EKF estimate degrades, motivating the particle filter.

![KF Non-Gaussian Noise](/assets/module-b/lecture-7/kf-non-gaussian.png)

## Particle Filter

### Idea
Instead of restricting ourselves to parametric distributions, use a sample-based representation.

- **Monte Carlo method:** rely on repeated random sampling to obtain numerical results
- **Advantage:** can approximate complicated distributions
- More samples → the sampled histogram converges to the true PDF (10 → 50 → 100 → 1000 samples)

### Representing a Distribution by Sampling

**Unweighted samples (accept-reject sampling):**
- Uniformly draw many particles; accept those that fall under the PDF
- Each particle has equal weight; probability is encoded by the density of particles
- Issue: inefficient, many particles get thrown away

**Weighted samples (importance sampling):**
- Draw samples from an easier proposal distribution $q$, then weight by the target $p$:

$$
w_i = \frac{p(x_i)}{q(x_i)}
$$

- The target distribution is approximated as a sum of weighted Dirac deltas:

$$
p(x) \approx \sum_{i=1}^{n} w^{(i)}\,\delta_{x^{(i)}}(x)
$$

![Sampling Methods](/assets/module-b/lecture-7/sampling-methods.png)

### Problem Setting
A set of $N$ weighted particles represents the posterior:

$$
S = \{\, \langle x^{[i]}, w^{[i]} \rangle \mid i = 1, \dots, N \,\}, \qquad p(x) \approx \sum_{i=1}^{N} w^{(i)}\,\delta_{x^{(i)}}(x)
$$

In localization:
- **Prediction** ← Odometry / Dead Reckoning
- **Correction** ← LiDAR / Scan Matching

### Step 1: Prediction (propagate the dynamics)
Push each particle through the system dynamics with noise:

$$
x_{k+1}^{(i)} = f\!\left(x_k^{(i)}, u_k\right) + \epsilon_k
$$

- $f(x_k, u_k)$: system dynamic function (same role as the action model)
- $x_k^{(i)}$: $i$-th particle at timestep $k$
- $\epsilon_k$: process noise

Equivalent to drawing samples from the proposal distribution:

$$
x_{k+1}^{[i]} \sim p\!\left(x_{k+1} \mid x_k, u_k\right)
$$

### Step 2: Correction (with the observation)
Given a new observation, update each particle's weight by the likelihood of that observation:

$$
w_{k+1}^{(i)} \propto P\!\left(o_{k+1} \mid x_{k+1}^{(i)}\right)
$$

In localization: update the particle cloud with the odometry motion, then run scan matching for each particle to determine weights.

### Scan Correlation
Score how well a particle's predicted scan matches the actual map using a 2D correlation:

$$
S = \frac{\sum_m \sum_n (A_{mn} - \bar{A})(B_{mn} - \bar{B})}{\sqrt{\left(\sum_m \sum_n (A_{mn} - \bar{A})^2\right)\left(\sum_m \sum_n (B_{mn} - \bar{B})^2\right)}}
$$

- $A$: actual map, $B$: occupancy map from the scan, with

$$
A_{mn} = \begin{cases} 1, & \text{wall} \\ 0, & \text{free space} \end{cases}
$$

- Update the weight by the score: $w_{t+1} \leftarrow w_t \times S$
- The scan that aligns with the map well gets a high correlation score → high weight (the best-aligned particle wins)

![Scan Correlation](/assets/module-b/lecture-7/scan-correlation.png)

### Step 3: Resampling
Without resampling, weight concentrates on a few particles while the rest carry negligible weight (*particle degeneracy*).

Resampling redraws particles in proportion to their weights:
- High-weight particles are duplicated; low-weight particles die off
- Over $N$ iterations the particle cloud concentrates around high-probability regions
- Result: particles track the true posterior efficiently (Courtesy: Thrun, Burgard, Fox)

![Resampling](/assets/module-b/lecture-7/resampling.png)

### Particle Filter Loop (Summary)
1. **Predict**: propagate each particle through the motion model + noise (odometry)
2. **Correct**: weight each particle by the observation likelihood (scan correlation vs. map)
3. **Resample**: redraw particles according to weight

## Interpretation of Bayes Rule (Mapping Example)
Using $\text{posterior} = \dfrac{\text{likelihood} \times \text{prior}}{\text{evidence}}$, $\quad P(B \mid A) = \dfrac{P(A \mid B)P(B)}{P(A)}$:

- $B$: there is an obstacle at $x = 2$
- $A$: the LiDAR detects the obstacle
- $P(A)$: uncertainty from the LiDAR measurement
- $P(A \mid B)$: uncertainty from the position of the car
- $P(B)$: prior knowledge (is there a wall at $x = 2$?)
- $P(B \mid A)$: given the LiDAR observation, the probability of $B$

Open question: how do we get the prior knowledge? → mapping.

## Localization: "Where am I?"
Fuse two noisy information sources to estimate pose:
- **IMU input** (odometry info) → prediction
- **LiDAR input** (observation info) → correction

Neither alone is 100% sure (we always have noise), so the output is a distribution $p(x)$ over pose, refined every prediction/correction cycle.

## Key Takeaways
- Dead reckoning drifts because model + measurement noise accumulates → estimate a distribution, not a point
- Bayes Rule: $\text{posterior} \propto \text{likelihood} \times \text{prior}$ (evidence = normalizer)
- The Markov property + Law of Total Probability turn state estimation into a recursive two-step filter
- Bayes filter = Prediction (action model) → Correction (sensor model), repeated; prediction spreads uncertainty, correction sharpens it
- KF/EKF/UKF assume Gaussian noise for an analytical solution; particle filters use weighted samples for non-Gaussian, complicated distributions
- Particle filter on F1TENTH: predict with odometry, correct via LiDAR scan correlation against the map, then resample