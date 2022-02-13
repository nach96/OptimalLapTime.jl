using Test
using OptimalLapTime

@test [1, 2] + [2, 1] == [3, 3]
@test OptimalLapTime.greet() == "Hello World"