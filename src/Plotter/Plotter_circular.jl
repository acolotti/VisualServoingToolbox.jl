function plotCameraTrajectories(traj::Vector{VS.VSTrajectory{VS.circular}}, P::AbstractMatrix = [0 0]; scale::Int=12)

    (xTraj,zTraj,xBegin,zBegin,xFinal,zFinal) = Uti.vectorizeTrajectories(traj);

    plt = Plots.plot(xTraj,zTraj,linecolor=:blue,aspect_ratio = 1,legend=:none);

    if P != [0 0]
        Plots.scatter!(plt,P[1,:], P[2,:], m=(10,:o,:black));
    end

    Plots.plot!(plt,xBegin,zBegin,m=(scale,:o),markercolor=:purple,linetype=:scatter,opacity=.1);
    Plots.plot!(plt,xFinal,zFinal,m=(scale,:o),markercolor=:blue,linetype=:scatter,opacity=.2);

    return plt;
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

function getCircularShapes(x::Vector{<:Real},z::Vector{<:Real},θ::Vector{<:Real},mSize::Real = 0.15)
    n = length(x);
    xS = Array{Float64}(undef,4*n);
    zS = Array{Float64}(undef,4*n);

    R(θ) = [cos(θ) sin(θ); -sin(θ) cos(θ)];

    for i = 1:n
        currentTriangle = mSize*R(θ[i])*[-1 1 0; -1 -1 1] + [x[i];z[i]]*[1 1 1];
        xS[4*i-3:4*i] = [currentTriangle[1,:]..., NaN];
        zS[4*i-3:4*i] = [currentTriangle[2,:]..., NaN];
    end

    return Plots.Shape(xS,zS);
end