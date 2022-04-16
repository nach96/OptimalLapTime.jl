############################################################################
########################       Track Object       ##########################
############################################################################
"""
Track: Object defining a racing track.
# Fields
- sectors: Array of sectors defining the track. [type,length/sweep,radius,direction]
- xmid: Vector of x location between sectors
- smid: Vector of s (global displacement) between sectors
- n: Track width.
"""
struct Track
    sectors
    xmid
    smid
    n
end

"""get_xsmid(sections)
Get x coordinates in between sectors.
Get global s in between sectors.
"""
function get_xsmid(sectors)
    x0 = [0.0,0.0,0.0,0.0]
    s0 = 0.0
    s1 = 0.0
    x = [x0]
    s = [s0]
    for section in sectors
        if section[1]==0
            s1 += section[2]
            x1 = straight(s1,0,x0,s0)
        else
            s1 += section[2]*section[3]
            x1 = arc(s1,section[3],section[4],x0,s0)
        end
        push!(x,x1)
        push!(s,s1)
        s0 = s1
        x0 = x1
    end
    return x,s
end
get_xmid(sectors) = get_xsmid(sectors)[1]
get_smid(sectors) = get_xsmid(sectors)[2]

function Track(sectors)
    x0,s0 = get_xsmid(sectors)
    n=1.5
    return Track(sectors,x0,s0,n)    
end
function Track(sectors,n)
    x0,s0 = get_xsmid(sectors)
    return Track(sectors,x0,s0,n)
end


#################################################################################
###########################    Track elements    ################################
#################################################################################

function straight(s,m,x0,s0=0)
    x0,y0,θ0 = x0
    s = s-s0
    θ = θ0 + m
    x = x0 + s*cos(θ)
    y = y0 + s*sin(θ)
    κ = 0
    return [x,y,θ,κ]
end

function arc(s,r,dir,x0,s0=0)
    x0,y0,θ0 = x0
    s = s-s0

    if dir == 0 #Anticlockwise
        xc = x0 - r*sin(θ0)
        yc = y0 + r*cos(θ0)
        offs = -pi/2
        θ = θ0 + s/r
        x = xc + r*cos(offs + θ)
        y = yc + r*sin(offs + θ)
        κ = 1/r
    else #Clockwise
        xc = x0 + r*sin(θ0)
        yc = y0 - r*cos(θ0)
        offs = pi/2
        θ = θ0 - s/r
        x = xc + r*cos(offs + θ)
        y = yc + r*sin(offs + θ) 
        κ = -1/r 
    end
    
    return [x,y,θ,κ]
end


############################################################################
########################       Track functions    ##########################
############################################################################
function get_x(track,s)
    sf = track.smid[end]
    s0 = track.smid
    x0 = track.xmid
    sectors = track.sectors
    i = 1

    i = findmin(abs.(s0.-s))[2]
    s>=s0[i] ? i=i : i -= 1

    if(s>=sf)
        return x0[end]
    end
    if sectors[i][1] == 0
        x,y,θ,κ = straight(s,0,x0[i],s0[i])
    else
        x,y,θ,κ = arc(s,sectors[i][3],sectors[i][4],x0[i],s0[i])
    end
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
function cartesian(track,s_)
    s,n,μ = s_
    xc,yc,θ = get_x(track,s)
    x = xc - n*sin(θ)
    y = yc + n*cos(θ)
    Ψ = θ + μ
    return [x,y,Ψ]
end

function track_coordinates(track)
    sf = track.smid[end]
    s = 0:0.1:sf
    x_log = []
    y_log = []
    θ_log = []
    for si in s
        x,y,θ = get_x(track,si)
        push!(x_log, x)
        push!(y_log, y)
        push!(θ_log,θ)
    end
    return [x_log, y_log,θ_log]
end

"""
Vehicle trajectory
"""
function traj_coordinates(track, s_)
    x_log=[]
    y_log= []
    s,n,θ = s_
    idx=1
    for si in s
        x,y = cartesian(track,[si,n[idx],θ[idx]])
        idx+=1
        push!(x_log, x)
        push!(y_log, y)
    end
    return [x_log,y_log]
end


function lanes_coordinates(track)
    sf = track.smid[end]
    s = 0:0.1:sf
    n = track.n
    x_left=[]
    y_left = []
    x_right=[]
    y_right=[]
    for si in s
        x_l,y_l = cartesian(track,[si,n,0])
        x_r,y_r= cartesian(track,[si,-n,0])
        push!(x_left, x_l)
        push!(y_left, y_l)
        push!(x_right, x_r)
        push!(y_right, y_r)
    end
    return [x_left,y_left,x_right,y_right]
end


############################################################################
########################       Track examples     ##########################
############################################################################
# Track descriptor information:
# [type,length/sweep,radius,direction]
# type:      0 - straight, 1 - corner
# direction: 0 - left, 1 - right
oval = [
        [0, 150, 0, -1],
        [1, pi / 2, 50, 0],
        [0, 100, 0, -1],
        [1, pi / 2, 90, 0],
        [0, 300, 0, -1],
        [1, pi / 2, 50, 0],
        [0, 100, 0, -1],
        [1, pi / 2, 90, 0],
        [0, 155, 0, -1],
    ]

ovalTrack = Track(oval)    
barcelona = [
        [0, 927, 0, -1],
        [1, 1.48, 40, 1],
        [0, 32, 0, -1],
        [1, 1.04, 50, 0],
        [0, 87, 0, -1],
        [1, 1.10, 104, 1],
        [0, 30, 0, -1],
        [1, 1.60, 150, 1],
        [0, 263, 0, -1],
        [1, 1.48, 44, 1],
        [0, 10, 0, -1],
        [1, 1.56, 100, 1],
        [0, 177, 0, -1],
        [1, 2.54, 42, 0],
        [0, 130, 0, -1],
        [1, 0.55, 150, 0],
        [0, 115, 0, -1],
        [1, 1.48, 52, 0],
        [0, 45, 0, -1],
        [1, 0.4, 100, 1],
        [0, 181, 0, -1],
        [1, 1.72, 85, 1],
        [0, 505, 0, -1],
        [1, 2.60, 34, 0],
        [0, 100, 0, -1],
        [1, 0.8, 30, 0],
        [0, 10, 0, -1],
        [1, 3.20, 60, 1],
        [0, 135, 0, -1],
        [1, 1.40, 30, 1],
        [0, 100, 0, -1],
        [1, 1.65, 25, 0],
        [0, 10, 0, -1],
        [1, 1.50, 33, 1],
        [0, 74.84, 0, -1],
        [1, 1.503, 90, 1],
        [0, 142.0, 0, -1],
    ]

barcelonaTrack=Track(barcelona,3)
chicane = [
    [0, 20, 0, -1],
    [1, pi/2, 20, 0],
    [0, 5, 0, -1],
    [1, pi/2, 20, 1],
    [0, 10, 0, -1]
]
chicaneTrack = Track(chicane,3)
straight_s = [
    [0,50,0,-1]
]
straightTrack = Track(straight_s)