
include("Optimization.jl")

"""
Cm: Function to generate starting state
Nm: Function to generate neighbourhood
Sm: Function to select a solution among the neighbourhood
c: Temperature. (Optim step)
L: Number of movements for every c
"""

"""
Cm: Function to generate starting solution
"""
function SA_Cm(track,car_p)
    return actions_centerline(track,car_p)
end

"""
Define starting value of c0. (slide 37)
"""
function SA_c0(track)
    c0 = track.smid[end]
    return c0
end
SA_cooling(c) = c*0.9 #(slide 38)
SA_len(c) = 10 #(slide 39)

"""
Nm: Function to generate neighbourhood
Modify random action from the vector before going out of the road.
"""
function SA_Nm(x,sol,OP)
    x_new = deepcopy(x) #Be careful! If you modify x_new, x will be modified. 
    i = findmin(abs.(OP.s_control.-sol.t[end]))[2]
    nv = rand(1:i)
    n2 = rand((1,2))
    if (n2 ==2) #Modify torque
        Δx = rand()*20-10
    else #modify steering angle
        Δx = rand()*0.05-0.025
    end
    #println("Generating neighbour. n2=",n2,"; nv=",nv)
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
        print("Better")
    else
        J_snew,J_tnew = evaluate(sol_new,OP)
        J_sref,J_tref = evaluate(sol_ref,OP)
        if J_snew > J_sref
            ΔE = (J_snew-J_sref)
            print("J_snew worse; ")
        elseif J_snew==J_sref
            ΔE = J_tnew-J_tref
            print("J_tnew worse; ")
        end
        P = exp(-ΔE/c) #Probability of accepting wrong solution [0:1]
        print("ΔE=", ΔE)
    end
    rand()<=P ? accepted=true : accepted=false
    println("; P=",P,"; Accepted=",accepted)
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
        println("Old actions=",x)
        x = x_opt
        println("New actions=",x)
        if(OP.Debug)
            log = Log_opt(x,sol_opt,evaluate(sol_opt,OP))
            push!(OP.Log,log)
            plot_op(sol_opt,OP)
        end
    end
    return x
end

function SA_stop_criteria(c)
    c<0.01 ? stop=true : stop=false
    return stop
end

"""
Simulated Annealing (slide 33)
"""
function SA(OP::OptimizationProblem)
    println("Setup optimization")
    sim_set = setup_simulation(OP)
    x_opt = SA_Cm(OP.track,OP.car_p)
    c=SA_c0(OP.track)
    L=SA_len(c)
    println("Start optimization loop")
    while SA_stop_criteria(c) == false
        x_opt =SA_cicle(c,L,x_opt,OP,sim_set)
        c=SA_cooling(c)
        L=SA_len(c)
        println("New step. c = ",c)
    end
    return x_opt
end
