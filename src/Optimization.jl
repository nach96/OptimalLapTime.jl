

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

function evaluation(OP::OptimizationProblem)
    tspam = (0.0,OP.track.smid[end])
    p = [OP.car_p, track]
    prob = ODEProblem(OP.model,OP.x0,tspam,p)

    sol = solve(prob; callback=cbs, alg = Tsit5(),reltol=1e-7,dtmax=1.0)

    return [eval_t,eval_s,eval_n]
end


############################################################################
####################        Optimization Algorithms         ################
############################################################################

function optimize(OP::OptimizationProblem,method)
    method(OP)
end


