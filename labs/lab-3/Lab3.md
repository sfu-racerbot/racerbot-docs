# Lab 3

### Scope

The scope of lab3 is to learn and experiment with the maths 
behind the **2D rigid body Transformations** and **PID control systems formula**. 
Although ros2 tf library already provide us static and dynamic transformations, we should still learn how under the hood works.

here are two provided lab code that can allow the ros2 car simulation drive automatically, you need to configure your
sim.yaml inside config and manually add pid_auto_drive node, along with the parameters and PID values,
the number provided below is not accurate and you may need to adjust yourself. (if still doesnt understand ros2, now is a good chance)

    control_rate_hz: 20.0
    goal_tolerance_m: 0.40

    max_linear_speed: 2.0
    max_angular_speed: 0.35

    # Distance PID
    dist_kp: 1.2
    dist_ki: 0.03
    dist_kd: 0.08

    # Yaw PID 
    yaw_kp: 2.4
    yaw_ki: 0.02
    yaw_kd: 0.12


    log_every_n: 5




**2D Rigid Body Transformations** (optional)
Hint: remember Rʳ²ᵣ₁ means the rotation 
matrix that rotates coordinates expressed in frame r1 into frame r2.
The composition of transforms steps are listed:
Inverse Matrices = Rʳ²ᵣ₁ = (Rʳ1ᵣ2)^-1 = (Rʳ1ᵣ2)^transpose

if there is angle then we need to first account for composition of rotations:
Rwᵣ2​ = Rwᵣ₁ * Rʳ²ᵣ₁. 


**.sh startup scripts** (optional)
Starting from this lab, startup script is recommended for simulations and repetitive trial and error,
setting up a startup script for running these program is essential. Try doing this!


