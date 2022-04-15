include("CarSim.jl")
include("BirdView.jl")
#using StaticArrays

############################################################################
####################            Control callback         ###################
############################################################################
#smid = chicaneTrack.smid
vδ=zeros(6)
vT=40.0*ones(6)
s_control = zeros(6)

function control_affect(integrator)
    s = integrator.t
    #i = indexin(s,s_control)
    i = findmin(abs.(s_control.-s))[2] # s may be inexact? Use nearest
    T = vT[i]
    δ = vδ[i]
    integrator.u[7] = δ
    integrator.u[8] = T
    #print("Triggered on s: ",s,". T= ", T, " ,δ= ",δ)
end
control_cb = PresetTimeCallback(s_control,control_affect)

############################################################################
#####################       Ending conditions       ########################
############################################################################

terminate_condition(u,t,integrator) = abs(u[2])-3
terminate_condition2(u,t,integrator) = u[4]
terminate_affect!(integrator) = terminate!(integrator)
terminate_cb = ContinuousCallback(terminate_condition,terminate_affect!)
terminate_cb2 = ContinuousCallback(terminate_condition2,terminate_affect!)


############################################################################
#########################       Simulation     ############################# 
############################################################################

function setup_sim()
    base_car_params = [800.0,50.0,1.2,1.5,9.81,4.0,4.0,1.0,0.0]
    p = [base_car_params, chicaneTrack]
    x0 = [0.0,0.0,0.0,6.0,0.0,0.0,0.0,40.0]
    tspam = (0.0,smid[end])  

    s_control[:] = chicaneTrack.smid
    control_cb = PresetTimeCallback(s_control,control_affect)
    cbs = CallbackSet(terminate_cb,terminate_cb2,control_cb)

    #prob = ODEProblem(vehicle_model_s,x0,tspam,base_car_params)
    prob = ODEProblem(model_s,x0,tspam,p)
    return prob
end

############################################################################
#####################     Control variables     ############################ 
############################################################################
function solve_test(prob)
    print("First simulation is slow:")
    vδ[:] = [0.0,0.115,0.0,-0.12,0.0,0.0]
    vT[:] = [40.0,40.0,40.0,40.0,40.0,40.0]
    @time sol = solve(prob; callback=cbs, alg = Tsit5(),reltol=1e-7,dtmax=1.0)
    return sol
end
prob = setup_sim()
sol = solve_test(prob)
print(s_control)
"""
print("Loop 100 simulations:")
@time for i =1:100
    solve(prob; callback=cbs, alg = Tsit5(),reltol=1e-7,dtmax=1.0)
    print(Threads.threadid())
end

print("100 simulations in paralell threads:")
@time Threads.@threads for i =1:100
    solve(prob; callback=cbs, alg = Tsit5(),reltol=1e-7,dtmax=1.0)
    print(Threads.threadid())
end
# I have set VSCode to use julia up to 6 Threads. Speed increase is only x2
"""

s = sol.t
n = sol[2,:]
θ = sol[3,:]
δ = sol[7,:]
T = sol[8,:]
s_ = [s,n,θ]


figure_t = plot_traj_on_track(s_,chicaneTrack)
display(figure_t)
