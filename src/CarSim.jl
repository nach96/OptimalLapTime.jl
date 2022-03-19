using DifferentialEquations
using Symbolics
using StaticArrays
include("Path.jl") #Path related functions
#include("BirdView.jl")

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
    s,n,μ,vx,vy,r,δ,T = x; 
    m,Iz,lR,lF,g,Ksf,Ksr,Cr,κ = p;

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

    return [ds,dn,dμ,dvx,dvy,dr,dδ,dT]
end

############################################################################
####################     Generate Space Domain Model     ###################
############################################################################

@variables s,n,μ,vx,vy,r,ts,δ,T
@variables m,Iz,lR,lF,g,Ksf,Ksr,Cr,κ

u = [δ,T]
x = [s,n,μ,vx,vy,r]
x = vcat(x,u) #Input variables included in the state vector. To be saved and able to modify them with callbacks.
p = [m,Iz,lR,lF,g,Ksf,Ksr,Cr,κ]

dx = vehicle_model(x,p,ts)

#Get derivatives from s. Chain rule: dx/dt = dx/ds·ds/dt
dxds = dx.*1/dx[1]
dxds[1] = 1/dx[1] #We dont need state s. (It is now the independent variable.) Set time as first state.

#Generate a function from symbolic expression
model_func = build_function(dxds,[s,n,μ,vx,vy,r,δ,T],[m,Iz,lR,lF,g,Ksf,Ksr,Cr,κ],s)


"""vehicle_model_s(x,u,s)
Vehicle ODE in Space Domain
# Arguments
    -x = [t,n,μ,vx,vy,r,δ,T]
        - t: Time
        - n,μ: Lateral displacement and heading in curvilinear coordinates.
        - vx,vy,r: Velocities in inertial frame.
        - δ,T: Steering angle and Driving Torque.
    - p = [m,Iz,lR,lF,g,Ksf,Ksr,Cr,κ]
    - s: Travelled distance along centerline. Independent variable.
"""
vehicle_model_s = eval(model_func[1])

"""
Model combining track and vehicle. Getting κ from track.
 - p = [car_p,track]
    - car_p = [m,Iz,lR,lF,g,Ksf,Ksr,Cr,κ]
"""
function model_s(x,p,s)
    track = p[2]
    car_p = p[1]
    car_p[end] = get_x(track,s)[4]
    vehicle_model_s(x,car_p,s)
end