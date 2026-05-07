# Lecture 5 Notes

**Lecture 5:** https://www.youtube.com/watch?v=5asfD-_Z9x8

## Obstacle Avoidance without a Map

**Goal:** Navigate while avoiding obstacles using only current sensor data (no map).

Previous method: **Wall Following**
- Works on simple tracks
- Fails with complex obstacles and gaps

**Reactive navigation:** Make decisions from real-time perception only.

## Follow the Gap (FTG)

**Challenge:** Avoid obstacles in real time using LiDAR.  
**Assignment:** Implement Follow-the-Gap in simulation and on the vehicle.

Key idea: Choose a direction with enough space for the car, not just the furthest point.

### Choosing a Direction from LiDAR

LiDAR provides:
- Distance measurements
- Angle for each ray

Important insight:
- Furthest single ray ≠ safest direction
- Car must fit through the space

### Defining a Gap

**Gap:** Sequence of LiDAR readings that are sufficiently far from obstacles.

Formal definition:
- At least **n consecutive points**
- Each distance ≥ threshold **t**

Example:
- n = 3
- t = 5 m

Robot drives toward the **center of the widest gap**.

### Basic Follow-the-Gap Algorithm

1. Read LiDAR scan
2. Identify valid gaps
3. Select widest gap
4. Steer toward its center

### Problems with Naive FTG

**Safety**
- Ignores vehicle size

**Motion constraints**
- Cars are non-holonomic (cannot move sideways)

**Visual: Largest Gap Failure**

![Largest Gap Failure](/assets/module-b/lecture-5/largest-gap-failure.png)

### Holonomic vs Non-Holonomic

**Holonomic:** Can move directly in any direction (e.g., omni-wheel robot)

**Non-Holonomic:** Motion constrained (e.g., car)
- Must steer to reposition
- Cannot move sideways

F1TENTH car is **non-holonomic**.

### Configuration Space (Safety Concept)

**Affordances:** The safe actions available to the robot given its size and the environment (e.g., whether a gap is wide enough to pass through).

Represent:
- Robot as circle with radius r
- Obstacles as circles

Then:
- Inflate obstacles by robot radius
- Treat robot as a point

Result: Passing between inflated obstacles guarantees no collision and correctly represents the robot’s affordances (what paths are physically possible).

## Improved FTG — Safety Bubble

Key idea: Avoid the **closest obstacle first**.

### Algorithm

1. Find nearest LiDAR point (closest obstacle)
2. Place safety bubble around it (radius slightly > car width)
3. Set all points inside bubble to 0 (blocked)
4. Find longest sequence of non-zero points (max gap)
5. Choose the 'best' point inside the gap

### Why Safety Bubble Works

- Encodes vehicle size
- Focuses on closest danger
- Adapts automatically as car approaches obstacle
- Faster and safer than naive FTG

### Wiggling Problem

Symptom:
- Car oscillates left/right (S-shaped path)

Cause:
- Always steering toward deepest point

Fix:
- Set maximum distance threshold
- Follow center of gap instead

Result:
- Smoother steering

## Disparity Extender (Major Improvement)

### Motivation

Even improved FTG can:
- Clip corners
- Misjudge safe paths

### Disparity Definition

A **disparity** occurs when adjacent LiDAR readings differ significantly.

Example: 2.2 m → 4.8 m

Meaning: Edge of an obstacle detected.

### Disparity Extender Algorithm

1. Detect **disparities** (large jumps in adjacent LiDAR distances)
2. For each disparity, **extend** the closer obstacle into free space by half the vehicle width
3. Replace affected LiDAR points with the closer distance (**virtual lidar readings**)
4. Choose direction with the furthest reachable safe distance

**Visual: Disparity Extender Approach**

![Disparity Extender Approach](/assets/module-b/lecture-5/disparity-extender-approach.png)

### Why It Works

- Accounts for vehicle size automatically
- Prevents corner collisions
- Produces smoother motion
- Computationally fast

### Performance Requirement

- LiDAR update rate: 40 Hz
- Time per update: 25 ms
- Processing target: < 10 ms per cycle

## Speed Control

Speed depends on obstacle distance.

General rule:
- Far obstacle → higher speed
- Near obstacle → lower speed

Typically implemented using: Piecewise linear function.

### Cornering Safety Rule

Problem:
Rear of car may hit obstacle during turns.

Solution:
- Check LiDAR readings behind and to the side
- If obstacle is too close in turning direction:
  - Stop turning
  - Drive straight

**Visual: Disparity Extender Cornering**

![Disparity Extender Cornering](/assets/module-b/lecture-5/disparity-extender-cornering.png)

### Known Failure Case — U-Turn Problem

Occurs when:
- Wide area behind car
- Narrow path ahead

Robot may: Turn the wrong direction.

Reason: Reactive methods lack global context.

This issue shouldn't occur on any track with a uniform width.

**Visual: Disparity Extender U-turn**

![Disparity Extender U-turn](/assets/module-b/lecture-5/disparity-extender-U-turn.png)

## Final Follow-the-Gap Pipeline

1. Read LiDAR scan
2. Find closest obstacle
3. Apply safety bubble
4. Find maximum gap
5. Choose target direction
6. Set steering angle
7. Set speed based on distance

## Other Reactive Navigation Methods (Overview)

### Bug Algorithms

Behavior:
- Move toward goal
- Follow wall if blocked

Limitation: Requires known goal location.

### Artificial Potential Fields

Idea:
- Goal attracts robot
- Obstacles repel robot
- Motion follows negative gradient

Major problem: **Local minima** (robot gets stuck).

## Key Takeaways

- Follow-the-Gap is simple, fast, and map-free
- Safety bubble accounts for vehicle size
- Disparity extender greatly improves robustness
- Direction choice depends on method:
  - FTG → drive toward center of the largest gap
  - Disparity Extender → drive toward furthest safe direction
- Reactive methods can achieve high racing performance
- Small algorithm tweaks significantly improve stability
