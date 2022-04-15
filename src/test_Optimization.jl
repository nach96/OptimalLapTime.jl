include("SimulatedAnnealing.jl")
include("BirdView.jl")
include("Path.jl")


car_p = [800.0,50.0,1.2,1.5,9.81,4.0,4.0,1.0,0.0]
x0 = [0.0,0.0,0.0,6.0,0.0,0.0,0.0,40.0]
eval_f = [eval_t, eval_s]
fig = plot()
debug = true
Log=[]
s_control=[] #Maybe it is a good idea to set manually the s_control before defining the problem?
actions=[[],[]]
opt_chicane = OptimizationProblem(  #inputs
                                    model_s,
                                    chicaneTrack,
                                    car_p,
                                    x0,
                                    eval_f,
                                    s_control,
                                    debug,
                                    #outputs
                                    actions,                                  
                                    fig,
                                    Log
)

#=
OP = opt_chicane
prob = setup_simulation(OP)
x=SA_Cm(OP.track,OP.car_p)
println(x)
sol=simulate(x,prob,OP)
=#

actions = optimize(opt_chicane,SA)

OP = opt_chicane
vx=[]
vsol=[]
vevals=[]
vevalt=[]
for step in OP.Log
    push!(vx,step.actions)
    push!(vsol,step.solutions)
    push!(vevals,step.eval_st[1])
    push!(vevalt,step.eval_st[2])
end
