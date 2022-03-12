include("SymCarSim.jl")

# Simulate t-model and s-model with this static input data. Plot both together.
T = ones(80)*10
δ = [zeros(50);ones(30)*0.1]

function static_control_vehicle(x,p,t)
    s,n,μ,vx,vy,r= x; 
    m,Iz,lR,lF,g,Ksf,Ksr,Cr,path,T,δ = p;
    cs = round(t)/10 #Control_step
    T_ = T[cs]
    δ_ = δ[cs]
    κ_ = path(s)[4]
    u = [T_,δ_,κ_]

    dx = vehicle_model(x,u,p,t)
    return dx
end

function static_car_sim()
    base_car_params = [800,50,1.2,1.5,9.81,4,4,1,path1,T,δ]
    x0=[0,0,0,6,0,0]
    tspam = (0.0,8.0)

    #End simulation if out of the road.
    function terminate_condition(u,t,integrator)
        abs(u[2])-3
        u[4]
    end
    terminate_affect!(integrator) = terminate!(integrator)
    terminate_cb = ContinuousCallback(terminate_condition,terminate_affect!)
    prob = ODEProblem(controlled_vehicle,x0,tspam,base_car_params)
    #prob = ODEProblem(ol_ctrl_vehicle,x0,tspam,base_car_params)

    @time global sol = solve(prob; callback=terminate_cb, alg = Tsit5(),reltol=1e-7,dtmax=0.01) #[ds,dn,dμ,dvx,dvy,dr]
    #return sol
    s = sol[1,:]
    n = sol[2,:]
    θ = sol[3,:]
    global s_ = [s,n,θ]
    global figure = plot_traj_on_track(s_,path1)
    display(figure)
end

static_car_sim()