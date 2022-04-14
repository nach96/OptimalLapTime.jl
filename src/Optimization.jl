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
struct OptimizationProblem
    model
    track
    car_p
    x0
    eval_functions
end


############################################################################
####################          Evaluation functions         #################
############################################################################
eval_t(sol) = sol[1,end]
eval_s(sol,track) = track.smid[end] - sol[1,end]
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

function simulate(OP::OptimizationProblem)
    tspam = (0.0,OP.track.smid[end])
    p = [OP.car_p, OP.track]
    prob = ODEProblem(OP.model,OP.x0,tspam,p)
    sol = solve(prob; callback=cbs, alg = Tsit5(),reltol=1e-7,dtmax=1.0)
    return sol
end

function evaluate(sol)
    J_t = eval_t(sol)
    J_s = eval_s(sol,OP.track)
    return [J_s,J_t]
end

function is_better_sol(sol_new,sol_ref)
    #TODO: Make generic function pareto_dominate(x,y)
    #= Diapo 58.
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
    J_tnew,J_snew = evaluate(sol_new)
    J_tref,J_sref = evaluate(sol_ref)
    better = false
    
    if J_snew < J_sref
        better = true
    if J_snew==J_sref and J_tnew<J_tref
        better = true
    end

    return better       
end
#Define operator << (Ex: sol1<<sol2 -> true/false)
# sol1<<sol2 = better_sol(sol1,sol2)


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
c: Temperature?
L: Length of neighbourhood?
"""

function SA_Cm(track)
    #Where do I decide how to divide the actions?
    # A good start might be constant force to keep speed and steering to theoretically follow the arc center.
    T = 40*ones(N)
    # delta = 0 if straight
    # delta = asin(L/R) if arc
    x[1] = T
    x[2] = delta
    return x
end

function Nm(x)

end
function metropolis(x,x_check)
    accepted = false
    evaluate(x)


    return accepted
end
function Sm(x)

end

function SA_process(Nm,c,L,x)
    for l in L:
        N = Nm(x)
        x_sel = Sm(x)
        #accept(x_sel,x) ? x=x_sel
    end
    return x
end

"""
Simulated Annealing
"""
function SA()
    x=Cm()
    c=Init(x)
    L=length(c)
    x_opt = x
    while stop_criteria == false
        x =SA_process(Nm,c,L,c)
        x < x_opt ? x=x_opt
        c=cooling(c)
        L=length(c)
    end
    return x
end

