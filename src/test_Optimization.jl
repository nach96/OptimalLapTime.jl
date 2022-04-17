include("SimulatedAnnealing.jl")
include("BirdView.jl")
include("Path.jl")


car_p = [800.0,50.0,1.2,1.5,9.81,4.0,4.0,1.0,0.0]
x0 = [0.0,0.0,0.0,6.0,0.0,0.0,0.0,40.0]
eval_f = [eval_t, eval_s]
fig = plot()
debug = true
Log=[]
track=barcelonaTrack
#s_control= chicaneTrack.smid #This should go on the constructor.
#s_control = [0,5,10,15,20,30,40,51.41,56.41,66,76,87.83,92,97.83]
s_tot = track.smid[end]
s_control=[0.0]
for i in 1:Int(round(s_tot/5))-1
    push!(s_control,s_control[i]+=5)
end
actions=[[],[]]
actions_start=[]
sa_par = SA_params(0.9,0.1,10,[0.05,10.0],actions_start)
opt_chicane = OptimizationProblem(  #inputs
                                    model_s,
                                    track,
                                    car_p,
                                    x0,
                                    eval_f,
                                    s_control,
                                    sa_par,
                                    debug,
                                    #outputs
                                    actions,                                  
                                    fig,
                                    Log
)

#=
OP = opt_chicane
prob = setup_simulation(OP)
x=SA_Cm(OP)
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
