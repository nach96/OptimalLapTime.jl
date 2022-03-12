using DifferentialEquations
include("Path.jl") #Path related functions
include("BirdView.jl")

############################################################################
########################       Time Domain Model  ##########################
############################################################################
"""vehicle_model(x,u,t)
Vehicle ODE in Time Domain
# Arguments
    -x = [s,n,μ,vx,vy,r]
        - s,n,μ = Absolute location in curvilinear coordinates.
        - vx, vy, r = Velocities in inertial frame
    - u = [δ, T] (Steering angle and Driving Torque).
    - p = m,lfr,lrr,
    - t = Time: Independent variable
    Example params: [500,20,2,1.5,9.81,1.2,1.2,1,path2]
"""
#function vehicle_model(x,u,p,t)
function vehicle_model(x,p,t)
    s,n,μ,vx,vy,r,δ,T,κ = x; 
    m,Iz,lR,lF,g,Ksf,Ksr,Cr = p;

    #Traction force, applied on center of gravity.
    Fx = T-Cr*vx^2; # TODO: Include rolling resisntance and aerodynamic drag. Fx = CmT − Cr0 − Cr2v2
    
    # Tire forces
    FNr = lR /(lF + lR )*m*g #Static load distribution. No load transfer.
    FNf = lF /(lF + lR )*m*g
    sideslip_f = atan(vy+lF*r,vx) - δ
    sideslip_r = atan(vy-lR*r, vx)
    Fyf = -FNf*Ksf*sideslip_f #Lateral force linear model
    Fyr = -FNr*Ksr*sideslip_r

    #ODE
    ds = (vx*cos(μ)-vy*sin(μ))/(1-n*κ);
    dn = vx*sin(μ) + vy*cos(μ);
    dμ = r - κ*ds;
    dvx = 1/m*(Fx - Fyf*sin(δ) + m*vy*r);
    dvy = 1/m*(Fyr + Fyf*cos(δ)-m*vx*r);
    dr = 1/Iz*(Fyf*lF*cos(δ)-Fyr*lR) # + Mtv); Mtv=Vectoring torque. =0 fixed axis.
    # Expanded state vector to include inputs. For saving and callback
    dδ=0
    dT=0
    dκ=0

    return [ds,dn,dμ,dvx,dvy,dr,dδ,dT,dκ]
end

#Simulate vehicle (time domain)
"""pure_pursuit()
"""
function pure_pursuit(s_,path;preview=3.0,K=0.1,δ_max=0.3,Kv=100.0)
    s,n,μ,vx = s_
    vehicle_pose = cartesian(s_,path)
    preview_pose = path(s+preview)
    dx = preview_pose[1]-vehicle_pose[1]
    dy = preview_pose[2]-vehicle_pose[2]
    α=atan(dy,dx)
    L=sqrt(dx^2+dy^2)
    Ly = L*sin(α-μ)
    δ = K*Ly
    -δ_max < δ < δ_max ? δ=δ : δ=sign(δ)*δ_max
    T = Kv*(10-vx)
    κ=path(s)[4]
    return SA[δ::Float64,T::Float64,κ::Float64]
end

function simulate_vehicle_t() 
    base_car_params = [800,50,1.2,1.5,9.81,4,4,1]
    x0=[0,0,0,6,0,0,0,0,0]
    tspam = (0.0,8.0)

    #Be carefull! Need to define pp parameters in a scope reachable by preiodic_pure_pursuit. I have been unable to put it as argument.
    pp = [0.2,4,0.3,40,path1]
    function periodic_pure_pursuit(integrator)
        #pp = [0.2,4,0.3,40,path1]

        K,preview,δ_max,Kv,path = pp
        s,n,μ,vx = integrator.u[1:4]
        u = pure_pursuit([s,n,μ,vx],path;K=K,preview=preview,δ_max=δ_max,Kv=Kv)
        
        integrator.u[7] = u[1]
        integrator.u[8] = u[2]
        integrator.u[9] = u[3]
        #integrator.u[7] = 0.0
        #integrator.u[8] = 0.0
        #integrator.u[9] = u[3]
    end
    control_cb = PeriodicCallback(periodic_pure_pursuit,0.1)

    #End simulation if out of the road or vehicle speed <=0
    terminate_condition(u,t,integrator) = abs(u[2])-3
    terminate_condition2(u,t,integrator) = u[4]
    terminate_affect!(integrator) = terminate!(integrator)
    terminate_cb = ContinuousCallback(terminate_condition,terminate_affect!)
    terminate_cb2 = ContinuousCallback(terminate_condition2,terminate_affect!)

    #cbs = CallbackSet(terminate_cb,terminate_cb2,control_cb)
    cbs = CallbackSet(terminate_cb2,control_cb)

    prob = ODEProblem(vehicle_model,x0,tspam,base_car_params)
    #prob = ODEProblem(ol_ctrl_vehicle,x0,tspam,base_car_params)

    @time global sol = solve(prob; callback=cbs, alg = Tsit5(),reltol=1e-7,dtmax=0.01) #[ds,dn,dμ,dvx,dvy,dr]
    #return sol
    s = sol[1,:]
    n = sol[2,:]
    θ = sol[3,:]
    δ = sol[7,:]
    T = sol[8,:]
    global s_ = [s,n,θ]
    global figure_t = plot_traj_on_track(s_,path1)
    display(figure_t)
    return sol
end

sol_t = simulate_vehicle_t()


#=
# Idea: Input symbolic values to time domain model, then derivatives will be symbolics as well, and apply chain rule to them to get space domain model.
# This should be pre-process or done every step??
# Then, only modifying time domain model, automatically you have space domain one.
"""vehicle_model(x,u,t)
Vehicle ODE in Space Domain
# Arguments
    - x = [s,n,μ,vx,vy,r]
        - s,n,μ = Absolute location in curvilinear coordinates.
        - vx, vy, r = Velocities in inertial frame
    - u = [δ, T] (Steering angle and Driving Torque).
    - t = Time: Independent variable
"""
function vehicle_model_s(x,u,p,s)
    
    t = 1/ds #Be carefull, singularity if car stops (ds =0) Start with some speed.

    ds,dn,dμ,dvx,dvy,dr = vehicle_model(x,u,p,t)
end
=#

