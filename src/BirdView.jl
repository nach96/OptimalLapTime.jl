using Plots
include("Path.jl")
#include("CarSim.jl")

############################################################################
########################       Plot Centerline    ##########################
############################################################################

function plot_centerline(path,figure=plot())
    x,y,θ = path_coordinates(path)
    figure = plot!(figure, 
                    x, y,
                    color = :black, 
                    linestyle = :dot,
                    lw=2,
                    label="Centerline")
    return figure
end

############################################################################
########################       Plot Lanes    ###############################
############################################################################



function plot_lanes(path,figure=plot())
    x_left, y_left, x_right, y_right = lanes_coordinates(path)
    figure = plot!(figure,x_left,y_left, color = :blue,label= "Lanes")
    figure = plot!(figure,x_right,y_right, color = :blue, label="Lanes")
    return figure
end

function plot_road(path)
    figure = plot_centerline(path)
    figure = plot_lanes(path,figure)
    return figure
end

############################################################################
####################       Plot vehicle trajectory    ######################
############################################################################


function plot_traj(s_,path,figure=plot())
    x,y= traj_coordinates(s_,path)
    figure = plot!(figure,x,y, color = :red, label="Trajectory")
    return figure
end

function plot_traj_on_track(s_,path,figure=plot())
    figure = plot_road(path)
    figure = plot_traj(s_,path,figure)
    return figure
end

############################################################################
####################           Test and Main          ######################
############################################################################

function test_plot_traj_on_path(path)
    s=0:0.1:100
    n = rand(-20:20,length(s))/10
    for i =1:length(n)-2
        n[i+2] = (n[i+2]+n[i+1]+n[i])/3
    end
    θ = zeros(length(s))
    figure = plot_traj_on_track([s,n,θ],path)
    return figure
end

#Show vehicle as triangle.
function plot_anim_traj(path)
    s=0:0.1:100
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
        plot_traj_on_track([s_anim,n_anim,θ_anim],path)
    end
end


function main()
    figure = test_plot_traj_on_path(path1)
    display(figure)
    #plot_anim_traj(path1)
end

#main()