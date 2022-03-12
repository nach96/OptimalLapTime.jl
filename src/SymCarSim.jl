using Symbolics
using StaticArrays
include("CarSim.jl")

#s-domain model. Symbolically obtained from time domain model:

@variables s,n,μ,vx,vy,r,ts,δ,T,κ
@variables m,Iz,lR,lF,g,Ksf,Ksr,Cr

u = [δ,T,κ]
x = [s,n,μ,vx,vy,r]
x = vcat(x,u) #Input variables included in the state vector. To be saved and able to modify them with callbacks.


p = [m,Iz,lR,lF,g,Ksf,Ksr,Cr]

dx = vehicle_model(x,p,ts)

#Get derivatives from s. Chain rule: dx/dt = dx/ds·ds/dt
dxds = dx.*1/dx[1]
dxds[1] = 1/dx[1] #We dont need state s. (It is now the independent variable.) Set time as first state.

#Generate a function from symbolic expression
model_func = build_function(dxds,[s,n,μ,vx,vy,r,δ,T,κ],[m,Iz,lR,lF,g,Ksf,Ksr,Cr],s)
vehicle_model_s = eval(model_func[1])

function test_time()
    base_car_params = [800.0,50.0,1.2,1.5,9.81,4.0,4.0,1.0]
    u0=[0.0,0.0,0.0]
    x0=[0.0,0.0,0.0,6.0,0.0,0.0]
    x0=vcat(x0,u0)
    tspam = (0.0,50.0)

    println("First time doesnt mind:")
    @time dx_ex = vehicle_model(x0,base_car_params,0.0)
    @time dxds_ex = vehicle_model_s(x0,base_car_params,0.0)

    println("Second time please work:")
    print("vehicle_model_t:")
    @time dx_ex = vehicle_model(x0,base_car_params,0.0)
    print("vehicle_model_s:")
    @time dxds_ex = vehicle_model_s(x0,base_car_params,0.0)


end

function simulate_vehicle_s()    
    base_car_params = [800.0,50.0,1.2,1.5,9.81,4.0,4.0,1.0]
    u0=[0.0,0.0,0.0]
    x0=[0.0,0.0,0.0,6.0,0.0,0.0]
    x0=vcat(x0,u0)
    tspam = (0.0,50.0)

    pp = [0.2,4.0,0.3,40.0,path1]
    function periodic_pure_pursuit(integrator)
        #pp = [0.2,4,0.3,40,path1]
        K,preview,δ_max,Kv,path = pp
        s = integrator.t
        n,μ,vx = integrator.u[2:4]
        u = pure_pursuit([s,n,μ,vx],path;K=K,preview=preview,δ_max=δ_max,Kv=Kv)

        #StaticArrays.setindex(integrator.u,u[1],7)
        #StaticArrays.setindex(integrator.u,u[2],8)
        #StaticArrays.setindex(integrator.u,u[3],9)

        integrator.u[7] = u[1]
        integrator.u[8] = u[2]
        integrator.u[9] = u[3]

        #integrator.u[7] = 0
        #integrator.u[8] = 0
        #integrator.u[9] = u[3]
    end
    control_cb = PeriodicCallback(periodic_pure_pursuit,0.1)

    #End simulation if out of the road or vehicle speed <=0
    terminate_condition(u,t,integrator) = abs(u[2])-3
    terminate_condition2(u,t,integrator) = u[4]
    terminate_affect!(integrator) = terminate!(integrator)

    function terminate_affect2!(integrator)
        println("Terminated because speed is 0")
        terminate!(integrator)
    end
    terminate_cb = ContinuousCallback(terminate_condition,terminate_affect!)
    terminate_cb2 = ContinuousCallback(terminate_condition2,terminate_affect2!)

    #cbs = CallbackSet(terminate_cb,terminate_cb2,control_cb)
    cbs = CallbackSet(terminate_cb2,control_cb)
    #cbs = CallbackSet(control_cb)

    prob = ODEProblem(vehicle_model_s,x0,tspam,base_car_params)
    @time sol = solve(prob; callback=cbs, alg = Tsit5(),reltol=1e-7,dtmax=1.0) #[ds,dn,dμ,dvx,dvy,dr]

    #return sol
    s = sol.t
    n = sol[2,:]
    θ = sol[3,:]
    δ = sol[7,:]
    T = sol[8,:]
    s_ = [s,n,θ]
    global figure_s = plot_traj_on_track(s_,path1,figure_t)
    display(figure_s)
    return sol
end

test_time()
sol_s = simulate_vehicle_s()
