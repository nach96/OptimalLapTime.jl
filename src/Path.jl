
#################################################################################
###########################    Path elements    #################################
#################################################################################

function straight(m,x0,s0,s)
    x0,y0,θ0 = x0
    s = s-s0
    θ = θ0 + m
    x = x0 + s*cos(θ)
    y = y0 + s*sin(θ)
    κ = 0
    return [x,y,θ,κ]
end

#TODO: Not working fine.
function arc(r,dir,x0_,s0,s_)
    x0,y0,θ0 = x0_
    s = s_-s0

    if dir == 0 #Anticlockwise
        xc = x0 - r*sin(θ0)
        yc = y0 + r*cos(θ0)
        offs = -pi/2
        θ = θ0 + s/r
        x = xc + r*cos(offs + θ)
        y = yc + r*sin(offs + θ)
    else #Clockwise
        xc = x0 + r*sin(θ0)
        yc = y0 - r*cos(θ0)
        offs = pi/2
        θ = θ0 - s/r
        x = xc + r*cos(offs + θ)
        y = yc + r*sin(offs + θ)
        
    end
    κ = 1/r
    return [x,y,θ,κ]
end

#################################################################################
#####################    Coordinates trasnsformations      ######################
#################################################################################
"""cartesian(s_,path)
# Arguments
    - s_ = [s,n,μ] Vehicle location in curvilinear coordinates
    - path = Reference path
# Output
    - [x,y,Ψ] Vehicle location in cartesian coordinates

"""
function cartesian(s_,path)
    s,n,μ = s_
    xc,yc,θ = path(s)
    x = xc - n*sin(θ)
    y = yc + n*cos(θ)
    Ψ = θ + μ
    return [x,y,Ψ]
end

function path_coordinates(path)
    s = 0:0.1:100
    x_log = []
    y_log = []
    θ_log = []
    for si in s
        x,y,θ = path(si)
        push!(x_log, x)
        push!(y_log, y)
        push!(θ_log,θ)
    end
    return [x_log, y_log,θ_log]
end

"""
Vehicle trajectory
"""
function traj_coordinates(s_,path)
    x_log=[]
    y_log= []
    s,n,θ = s_
    idx=1
    for si in s
        x,y = cartesian([si,n[idx],θ[idx]],path)
        idx+=1
        push!(x_log, x)
        push!(y_log, y)
    end
    return [x_log,y_log]
end

function lanes_coordinates(path)
    s = 0:0.1:100
    n = 1.5
    x_left=[]
    y_left = []
    x_right=[]
    y_right=[]
    for si in s
        x_l,y_l = cartesian([si,n,0],path)
        x_r,y_r= cartesian([si,-n,0],path)
        push!(x_left, x_l)
        push!(y_left, y_l)
        push!(x_right, x_r)
        push!(y_right, y_r)
    end
    return [x_left,y_left,x_right,y_right]
end

#################################################################################
###########################    Path examples    #################################
#################################################################################

"""path1(s)
# Arguments
    - s = Path distance traveled
Return [x,y,θ] for a given s.
Built with straights and arcs.
TODO: Fix arcs. Something not working fine.
"""
function path1(s)
    #Define road sections
    m0=0        #Straight
    r1=10        #Circle
    #m2=pi/2     #Stright
    m2=0

    s0 = 0
    s1 = s0 + 10
    s2 = s1 + r1*pi/2 
    s3 = s2 + 1
    s4 = s3 + r1*pi/2
    s5 = s4 + 20

    #Pre-calculate end points of every step:
    x0 = [0,0,0]
    x1 = straight(m0,x0,s0,s1)
    x2 = arc(r1,0,x1,s1,s2)
    x3 = straight(m2,x2,s2,s3)
    x4 = arc(r1,1,x3,s3,s4)
    x5 = straight(m0,x4,s4,s5)

    if s<s1
        x,y,θ,κ = straight(m0,x0,s0,s)
    elseif s<s2
        x,y,θ,κ = arc(r1,0,x1,s1,s)
    elseif s<s3
        x,y,θ,κ = straight(m2,x2,s2,s)
    elseif s<s4
        x,y,θ,κ = arc(r1,1,x3,s3,s)
    elseif s<s5
        x,y,θ,κ = straight(m0,x4,s4,s)
    else
        x,y,θ,κ = x4
    end
    return [x,y,θ,κ]
end

"""path2(s)
Return [x,y,θ] for a given s.
Built only with straight lines
"""
function path2(s)
    m0 = 0
    m1 = pi/4
    m2 = pi/4
    m3 = pi/4
    m4 = pi/4

    s0 = 0
    s1 = s0 + 10
    s2 = s1 + 2
    s3 = s2 + 8
    s4 = s3 + 2
    s5 = s4 + 10

    x0 = [0,0,0]
    x1 = straight(m0,x0,s0,s1)
    x2 = straight(m1,x1,s1,s2)
    x3 = straight(m2,x2,s2,s3)
    x4 = straight(m3,x3,s3,s4)
    x5 = straight(m4,x4,s4,s5)

    if s<s1
        x,y,θ,κ = straight(m0,x0,s0,s)
    elseif s<s2
        x,y,θ,κ = straight(m1,x1,s1,s)
    elseif s<s3
        x,y,θ,κ = straight(m2,x2,s2,s)
    elseif s<s4
        x,y,θ,κ = straight(m3,x3,s3,s)
    elseif s<s5
        x,y,θ,κ= straight(m4,x4,s4,s)
    else
        x,y,θ,κ = x5
    end
    return [x,y,θ,κ]
end

function f_path3(s)
    r0=20
    
    s0 = 0
    s1 = 2*pi*r0

    x0 =[0,0,0]
    x1 = arc(r0,0,x0,s0,s1)
    if(s<s1)
        x,y,θ,κ = arc(r0,1,x0,s0,s)
    else
        x,y,θ,κ = x1
    end
    return [x,y,θ,κ]
end

#TODO:
# Make paths way more generic. Idea:

"""
struct path3
    parts = [0, 1, 0, 2] #0=straight, 1=arc Clockwise; 2=arc counterclw
    coefs = [m0,r1,...]
    len_s = [s0,s1,...]
    s_max = sum(len_s)
end

#generic path(data,s) function
function path(data,s)

end
"""