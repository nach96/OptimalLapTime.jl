include("Optimization.jl")

car_p = [800.0,50.0,1.2,1.5,9.81,4.0,4.0,1.0,0.0]
x0 = [0.0,0.0,0.0,6.0,0.0,0.0,0.0,40.0]
eval_f = [eval_t, eval_s]
opt_chicane = OptimizationProblem(vehicle_model_s,
                                    chicaneTrack,
                                    car_p,
                                    x0,
                                    eval_f)
                                    