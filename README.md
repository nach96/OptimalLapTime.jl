# OptimalLapTime.jl

Documentation: https://nach96.github.io/OptimalLapTime.jl/dev/

This is a Learning Project.

Developing a library in Julia for Race Car Optimal Lap Time simulation, based on Heuristic Optimization.

# Development Steps:

You **MUST** finish the first item of every chapter before going to any second item. 

1. Develop vehicle model.
    1. Basic bycycle model. Solved by DifferentialEquations.  
2. Develop road model.
    1. 2D, equal road width in all track., only staights and arcs.
    2. Making it easy to build new tracks.
    3. Including more road details: slope, variable width ...
    4. Follow openDRIVE standard?
3. Visualize track and vehicle birdView.
    1. Offline. First simulate then plot.
    2. Online. Plot during simulation.
4. Implement PurePursuit Controller to drive along the road.
5. Develop interface Optimization-simulation.
6. Develop OHA algorithms to run within API 5
    1. Local Search algorithms:
        1. Simulated Annealing (SA)
        2. Tabú Search (TS)
    2. Evolutive techniques
        1. Genetic Algorithm (GA)
        2. Particle Swarm Optimization (PSO)
