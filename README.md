# OptimalLapTime.jl

<!-- Documentation: https://nach96.github.io/OptimalLapTime.jl/dev/" -->

Developing a library in Julia for Race Car Optimal Lap Time simulation, based on Heuristic Optimization.

# Vehicle model:
    - 3 DOF bycicle model in curvilinear coordinates (Inertial reference frame along track centerline)
    - Model simulated in s-domain. (s: centerline distance as independent variable, instead of time)
    - Inputs: Steering Angle and Traction Force.
# Track model:
    - Flat road, composed by sectors of straights and arcs.
# Optimization algorithms:
    - Simulated Annealing.
    Objective functions: 
        - J1 = s_end-s (Leftover distance when crash. Minimum 0)
        - J2 = t (Lap Time)

    

