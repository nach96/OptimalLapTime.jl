include("CarSim.jl")
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
    OP.s_control = OP.track.smid #This should go on the constructor.
    OP.actions[1] = zeros(length(OP.s_control)) #idem
    OP.actions[2] = zeros(length(OP.s_control))
    function control_affect(integrator)
        s = integrator.t
        i = findmin(abs.(OP.s_control.-s))[2]
        δ = OP.actions[1][i]
        T = OP.actions[2][i]
        integrator.u[7] = δ
        integrator.u[8] = T
        println("New action. s=",s,"; δ=",δ,"; T=",T)
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
    J_t = eval_t(sol)
    J_s = eval_s(sol,OP.track)
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
    J_tnew,J_snew = evaluate(sol_new,OP)
    J_tref,J_sref = evaluate(sol_ref,OP)
    better = false
    
    if J_snew < J_sref
        better = true
    elseif J_snew==J_sref && J_tnew <J_tref
        better = true
    end
    return better       
end

############################################################################
####################        Optimization Algorithms         ################
############################################################################

function optimize(OP::OptimizationProblem,method)
    method(OP)
end

"""
Cm: Function to generate starting state
Nm: Function to generate neighbourhood
Sm: Function to select a solution among the neighbourhood
c: Temperature. (Optim step)
L: Number of movements for every c
"""

#TODO: Here initial actions only for  the midle sections, but could be the same for an arbitrary s_control vector.
function actions_centerline(track,car_p)
    vs = track.smid
    s_control = vs
    vκ = getindex.(track.xmid,4)
    popfirst!(vκ)
    push!(vκ,0.0)
    L = car_p[3]+car_p[4]
    vδ = vκ*L   #Calculate steering from geometry, considering no sliip.
    vT = 40*ones(length(vs))
    return [vδ,vT]
end


function SA_Cm(track,car_p)
    #Where do I decide how to divide the actions?
    return actions_centerline(track,car_p)
end

# c0 = eval_max - eval_min (diapo 37)
# Considering only first pareto front: traveled distance s.
function SA_c0(track)
    c0 = track.smid[end]
    return c0
end
SA_cooling(c) = c*0.95 #(diapo 38)
SA_len(c) = 10 #(diapo 39)

#Neighbour generation. Check NextSteps.md
#Generate only one variation, and select it
function SA_Nm(x,sol)
    x_new = copy(x) #Be careful! If you modify x_new, x will be modified. 
    i = findmin(abs.(OP.s_control.-sol.t[end]))[2]
    nv = rand(1:i)
    n2 = rand((1,2))
    if (n2 ==2) #Modify torque
        Δx = rand()*20-10
    else #modify steering angle
        Δx = rand()*0.1-0.05
    end
    println("Generating neighbour. n2=",n2,"; nv=",nv)
    x_new[n2][nv]+= Δx
    return x_new
end
#acceptance criteria. Diapo 32 y 61
function metropolis(c,sol_new,sol_ref,OP)
    accepted = false
    if is_better_sol(sol_new,sol_ref,OP)
        #accepted = true
        P=1
    else
        J_tnew,J_snew = evaluate(sol_new,OP)
        J_tref,J_sref = evaluate(sol_ref,OP)
        if J_snew > J_sref
            ΔE = (J_snew-J_sref)*10
        elseif J_snew==J_sref
            ΔE = J_tnew-J_tref
        end
        P = exp(-ΔE/c) #Probability of accepting wrong solution [0:1] 
    end
    rand()<=P ? accepted=true : accepted=false
    println("P=",P,"; Accepted=",accepted)
    return accepted
end

#(diapo 33)
function SA_process(c,L,x,OP,sim_set)
    println("Simulate")
    sol=simulate(x,sim_set,OP)
    sol0 = deepcopy(sol)
    x0 = deepcopy(x)
    for l in L
        println("New iteration.")
        x_new = SA_Nm(x,sol)
        sol_new = simulate(x_new,sim_set,OP)
        if metropolis(c,sol_new,sol,OP) 
            x=x_new
            sol=sol_new
        end
    end
    if is_better_sol(sol,sol0,OP)!
        x=x0
    else
        println("Accepted solution. New actions=",x)
        if(OP.Debug)

            log = Log_opt(x,sol,evaluate(sol,OP))
            push!(OP.Log,log)
            plot_op(sol,OP)
        end
    end
    return x
end

function SA_stop_criteria(c)
    stop = false
    c<0.1 ? stop=true : stop=false
    return stop
end

"""
Simulated Annealing
"""
#(diapo 33)
function SA(OP::OptimizationProblem)
    println("Setup optimization")
    sim_set = setup_simulation(OP)
    x=SA_Cm(OP.track,OP.car_p)
    c=SA_c0(OP.track)
    L=SA_len(c)
    x_opt = x
    println("Start optimization loop")
    while SA_stop_criteria(c) == false
        x_opt =SA_process(c,L,x,OP,sim_set)
        c=SA_cooling(c)
        L=SA_len(c)
        println("New step. c = ",c)
    end
    return x_opt
end

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
