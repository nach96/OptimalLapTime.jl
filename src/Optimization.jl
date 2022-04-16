include("CarSim.jl")
"""
Base Optimization functions.
"""
############################################################################
################     Generic Optimization Problem          #################
############################################################################
"""
OptimizationProblem
# Fields
    - model: Model used for simulation
    - track
    - car_p: [m,Iz,lR,lF,g,Ksf,Ksr,Cr,κ]
    - x0: [t,n,μ,vx,vy,r,δ,T]
    - eval_functions: Array of different objetive funtions.
"""
mutable struct OptimizationProblem
    #Inputs
    model
    track
    car_p
    x0
    eval_functions
    s_control
    opt_params
    Debug #flag (=1 to plot and log)
    #Outputs
    actions
    fig
    Log #::Vector{Log_opt}
end
#TODO: Constructor with the minimum params. Set the calculated values inside.
#Is it okey to have inputs and outputs in the same struct? Or is cleaner to set two.
struct Log_opt
    actions
    solutions
    eval_st #::eval_st
end
struct eval_st
    eval_s
    eval_t
end

############################################################################
####################        Optimization Algorithms         ################
############################################################################
function optimize(OP::OptimizationProblem,method)
    method(OP)
end

############################################################################
#####################            Simulation         ########################
############################################################################
function setup_simulation(OP::OptimizationProblem)
    #Terminal conditions
    terminate_condition(u,t,integrator) = abs(u[2])-3
    terminate_condition2(u,t,integrator) = u[4]
    terminate_affect!(integrator) = terminate!(integrator)
    terminate_cb = ContinuousCallback(terminate_condition,terminate_affect!)
    terminate_cb2 = ContinuousCallback(terminate_condition2,terminate_affect!)

    #Control
    OP.actions[1] = zeros(length(OP.s_control)) #idem
    OP.actions[2] = zeros(length(OP.s_control))
    function control_affect(integrator)
        s = integrator.t
        i = findmin(abs.(OP.s_control.-s))[2]
        δ = OP.actions[1][i]
        T = OP.actions[2][i]
        integrator.u[7] = δ
        integrator.u[8] = T
        #println("New action. s=",s,"; δ=",δ,"; T=",T)
    end
    control_cb = PresetTimeCallback(OP.s_control,control_affect)
    cbs = CallbackSet(terminate_cb,terminate_cb2,control_cb)
    tspam = (0.0,OP.track.smid[end])
    p = [OP.car_p, OP.track]
    prob = ODEProblem(OP.model,OP.x0,tspam,p)
    return [prob,cbs]
end

function simulate(x,sim_set,OP)
    OP.actions[1][:] = x[1]
    OP.actions[2][:] = x[2]
    prob=sim_set[1]
    cbs=sim_set[2]
    sol = solve(prob; callback=cbs, alg = Tsit5(),reltol=1e-7,dtmax=1.0)
    return sol
end
############################################################################
####################          Evaluation functions         #################
############################################################################
#Evaluation functions
eval_t(sol) = sol[1,end]
eval_s(sol,track) = track.smid[end] - sol.t[end]
function eval_n(sol,track,Kn)
    s = sol[1,:]
    n = sol[2,:]
    cost = 0
    for si in s
        if n > track.n
            cost += Kn*abs(n[si]-track.n)
        end
    end
    return cost
end
function evaluate(sol,OP)
    J_s = eval_s(sol,OP.track)
    J_t = eval_t(sol)
    return [J_s,J_t]
end
function is_better_sol(sol_new,sol_ref,OP)
    #=TODO: Make generic function pareto_dominate(x,y) Diapo 58.
        evx = evaluate(sol)
        ecy = evaluate(sol)
        one_improves = false
        for evx,evy in ev_x, evy
            if evx > evy
                return false
            end
            if ev(x) < ev(y)
                one_improves = true
            end
        end
        return one_improves
    =#
    J_snew,J_tnew = evaluate(sol_new,OP)
    J_sref,J_tref = evaluate(sol_ref,OP)
    better = false
    
    if J_snew < J_sref
        better = true
    elseif J_snew==J_sref && J_tnew <J_tref
        better = true
    end
    return better       
end

############################################################################
####################           Initial solution            #################
############################################################################
#TODO: Here initial actions only for  the midle sections, but could be the same for an arbitrary s_control vector.
function actions_centerline(track,car_p,s_control)
    vκ=[]
    for si in s_control
        push!(vκ,get_x(track,si)[4])
    end
    L = car_p[3]+car_p[4]
    vδ = vκ*L   #Calculate steering from geometry, considering no sliip.
    vT = 40*ones(length(s_control))
    return [vδ,vT]
end

############################################################################
#########################           Plot OP              ###################
############################################################################
function plot_op(sol,OP)
    s = sol.t
    n = sol[2,:]
    θ = sol[3,:]
    δ = sol[7,:]
    T = sol[8,:]
    s_ = [s,n,θ]
    #fig = plot_traj_on_track(s_,OP.track,fig)
    OP.fig = plot_traj_on_track(s_,OP.track,OP.fig)
    display(OP.fig)
end
