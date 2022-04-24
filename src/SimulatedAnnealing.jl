
include("Optimization.jl")
include("utils.jl")
#Verbose: true=print msgs.
vb=false

"""SA_params
# Fields
    - kc: Cooling coeficent (0.8-0.95)
    - c_end: Final "temperature" (0.1)
    - L:    Optimization batch (9)
    - x_max: [δ_max, T_max]
    - x_0: Initial solution. If [], will calculate centeline.
"""
struct SA_params
    kc      
    c_end   
    L       
    x_max
    x_start   
end

"""
Cm: Function to generate starting solution
"""
function SA_Cm(OP::OptimizationProblem)
    if OP.opt_params.x_start==[]
        x0= actions_centerline(OP.track,OP.car_p,OP.s_control)
    else
        x0 = OP.opt_params.x_start
    end
    return x0
end

"""
Define starting value of c0. (slide 37)
"""
function SA_c0(OP::OptimizationProblem)
    c0 = OP.track.smid[end]
    return c0
end
SA_cooling(c,par::SA_params) = c*par.kc #(slide 38)
SA_len(par::SA_params) = par.L #(slide 39)

"""
Nm: Function to generate neighbourhood
Modify random action from the vector before going out of the road.
"""
function SA_Nm(x,sol,OP::OptimizationProblem)
    x_new = deepcopy(x) #Be careful! If you modify x_new, x will be modified. 
    i = findmin(abs.(OP.s_control.-sol.t[end]))[2]
    i < length(OP.track.smid) ? i_min = max(1,i-3) : i_min=1
    nv = rand(i_min:i)
    n2 = rand((1,2))
    #Modify action
    Δx = rand()*OP.opt_params.x_max[n2]-OP.opt_params.x_max[n2]/2  
    dprintln(vb,"Generating neighbour. n2=",n2,"; nv=",nv)
    x_new[n2][nv]+= Δx
    return x_new
end

"""
metropolis algorithm for acceptance criteria of a solution (slides 32 and 61)
"""
function metropolis(c,sol_new,sol_ref,OP)
    P=0
    if is_better_sol(sol_new,sol_ref,OP)
        P=1
        dprint(vb,"Better")
    else
        J_snew,J_tnew = evaluate(sol_new,OP)
        J_sref,J_tref = evaluate(sol_ref,OP)
        if J_snew > J_sref
            ΔE = (J_snew-J_sref)
            dprint(vb,"J_snew worse; ")
        elseif J_snew==J_sref
            ΔE = J_tnew-J_tref
            dprint(vb,"J_tnew worse; ")
        end
        P = exp(-ΔE/c) #Probability of accepting wrong solution [0:1]
        dprint(vb,"ΔE=", ΔE)
    end
    rand()<=P ? accepted=true : accepted=false
    dprintln(vb,"; P=",P,"; Accepted=",accepted)
    return accepted
end

"""
Step of Simulated Annealing algorithm (slide 33)
"""
function SA_cicle(c,L,x,OP,sim_set)
    sol=simulate(x,sim_set,OP)
    sol_opt = deepcopy(sol)
    x_opt = deepcopy(x)
    for l in 1:L
        x_new = SA_Nm(x_opt,sol_opt,OP)
        sol_new = simulate(x_new,sim_set,OP)
        if metropolis(c,sol_new,sol_opt,OP) 
            x_opt=deepcopy(x_new)
            sol_opt=deepcopy(sol_new)
        end
    end
    if is_better_sol(sol_opt,sol,OP)
        dprintln(vb,"Old actions=",x)
        x = x_opt
        dprintln(vb,"New actions=",x)
        if(OP.Debug)
            log = Log_opt(x,sol_opt,evaluate(sol_opt,OP))
            push!(OP.Log,log)
            plot_op(sol_opt,OP)
        end
    end
    return x
end

function SA_stop_criteria(c,par::SA_params)
    c<par.c_end ? stop=true : stop=false
    return stop
end

"""
Simulated Annealing (slide 33)
"""
function SA(OP::OptimizationProblem)
    dprintln(vb,"Setup optimization")
    sim_set = setup_simulation(OP)
    x_opt = SA_Cm(OP)
    c=SA_c0(OP)
    L=OP.opt_params.L
    println(vb,"Start optimization loop")
    while SA_stop_criteria(c,OP.opt_params) == false
        x_opt =SA_cicle(c,L,x_opt,OP,sim_set)
        c=SA_cooling(c,OP.opt_params)
        L=OP.opt_params.L
        println("New step. c = ",c)
    end
    return x_opt
end
