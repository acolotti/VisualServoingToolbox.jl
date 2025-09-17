function plotCameraTrajectories(traj::Vector{VS.VSTrajectory{VS.circular}}, P::AbstractMatrix = [0 0]; point_size::Real = 10, scale::Int=12)
    fig = GLM.Figure();
    ax = GLM.Axis(fig[1,1], title = "", aspect = GLM.DataAspect());

    if P != [0 0]
        GLM.scatter!(ax, P[1,:], P[2,:], markersize = point_size, color = :black); # plots tracking points
    end

    for t in traj
        GLM.lines!(ax,map(e->GLM.Point2f(e[1:2]),t.u),color=:blue);
        GLM.scatter!(ax, GLM.Point2f.([t.u[1],t.u[end]]), markersize = scale, color = [GLM.RGBA{Float32}(0, 0, 1, 0.1),GLM.RGBA{Float32}(160/255, 32/255, 240/255, 0.2)]);
    end

    return fig;
end

function animateTrajectories(traj::Vector{VS.VSTrajectory{VS.circular}}, P::AbstractMatrix = [0 0]; scale::Int=15, filename::String = "", duration::Int = 10, simEndTime::Int = 0)
    if filename == ""
        filename = "videos/circular_"*Dates.format(Dates.now(), "yyyymmdd-HH:MM")*".mp4";
    end
    
    n = length(traj);
    intTrs = Vector{TrajectoryInterpolator}(undef,n);

    xlim = [0,0];
    ylim = [0,0];

    tmax = 0;

    for i = 1:n
        intTrs[i] = TrajectoryInterpolator(traj[i].u,traj[i].t);
        if simEndTime == 0
            tmax = max(maximum(traj[i].t),tmax);
        end
    end

    if simEndTime != 0
        tmax = simEndTime;
    end

    time = GLM.Node(0.0);

    trs = GLM.@lift [intTrs[i]($time) for i=1:n]
    x = GLM.@lift [$trs[i][1] for i=1:n]
    z = GLM.@lift [$trs[i][2] for i=1:n]

    tracks = [GLM.Node(GLM.Point2f[(GLM.to_value(x)[i],GLM.to_value(z)[i])]) for i = 1:n]

    GLM.set_theme!(GLM.theme_black())

    fig = GLM.lines(tracks[1], colormap = :inferno, linewidth = 2,
        axis = (title = GLM.@lift("t = $(round($time, digits = 1))"), aspect = GLM.DataAspect(),)) #limits=(0,8,0,5),
    for i = 2:n
        GLM.lines!(tracks[i], linewidth = 2)
    end
    GLM.scatter!(x, z, color = :red, markersize = scale)
    GLM.scatter!(P[1,:], P[2,:], color = :white, markersize = 15)

    framerate = 30
    timestamps = range(0, tmax, length = framerate*duration)

    GLM.record(fig, filename, timestamps;
            framerate = framerate) do t
                
        time[] = t
        for i = 1:n
            new_point = GLM.Point2f(GLM.to_value(x)[i], GLM.to_value(z)[i])
            push!(tracks[i][], new_point)
            notify(tracks[i])
        end

    end

end

function potentialVisualization(::typeof(VS.circular),P::AbstractMatrix,sd::AbstractMatrix,xrange::AbstractVector,zrange::AbstractVector; points::Integer=1000)
    e(x,z) = VS.getError(VS.circular,[x;z],[P;sd])[1,1];
    x = range(xrange[1],xrange[2],length=points);
    z = range(zrange[1],zrange[2],length=points);

    GLM.set_theme!(GLM.theme_light());
    GLM.surface(x,z,e);    
end