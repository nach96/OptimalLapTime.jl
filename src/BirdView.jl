using Plots
include("Path.jl")
#include("CarSim.jl")

############################################################################
########################       Plot Centerline    ##########################
############################################################################

function plot_centerline(track,figure=plot())
    x,y,θ = track_coordinates(track)
    figure = plot!(figure, 
                    x, y,
                    color = :black, 
                    linestyle = :dot,
                    lw=2,
                    label=false)
                    #label="Centerline")
                    
    return figure
end

############################################################################
########################       Plot Lanes    ###############################
############################################################################



function plot_lanes(track,figure=plot())
    x_left, y_left, x_right, y_right = lanes_coordinates(track)
    figure = plot!(figure,x_left,y_left, color = :blue,label=false)#,label= "Lanes")
    figure = plot!(figure,x_right,y_right, color = :blue,label=false)#, label="Lanes")
    return figure
end

function plot_road(track,figure=plot())
    figure = plot_centerline(track,figure)
    figure = plot_lanes(track,figure)
    return figure
end

############################################################################
####################       Plot vehicle trajectory    ######################
############################################################################


function plot_traj(s_,track,figure=plot())
    x,y= traj_coordinates(track,s_)
    figure = plot!(figure,x,y, color = :red,label=false)#, label="Trajectory")
    return figure
end

function plot_traj_on_track(s_,track,figure=plot())
    figure = plot_road(track,figure)
    figure = plot_traj(s_,track,figure)
    return figure
end

############################################################################
####################           Test and Main          ######################
############################################################################

function test_plot_traj_on_track(track)
    sf = track.smid[end]
    s=0:0.1:sf
    n = rand(-20:20,length(s))/10
    for i =1:length(n)-2
        n[i+2] = (n[i+2]+n[i+1]+n[i])/3
    end
    θ = zeros(length(s))
    figure = plot_traj_on_track([s,n,θ],track)
    return figure
end

#Show vehicle as triangle.
function plot_anim_traj(track)
    sf = track.smid[end]
    s=0:0.1:sf
    n = rand(-20:20,length(s))/10
    for i =1:length(n)-2
        n[i+2] = (n[i+2]+n[i+1]+n[i])/3
    end
    θ = zeros(length(s))
    s_anim=[]
    n_anim=[]
    θ_anim=[]
    anim = @animate for i=1:length(n)
        push!(s_anim,s[i])
        push!(n_anim,n[i])
        push!(θ_anim,n[i])
        plot_traj_on_track([s_anim,n_anim,θ_anim],track)
    end
end


function main()
    figure = test_plot_traj_on_track(chicaneTrack)
    display(figure)
    #plot_anim_traj(path1)
end

#main()