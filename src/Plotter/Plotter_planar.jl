function plotCameraTrajectories(traj::Vector{VS.VSTrajectory{VS.planar}}, P::AbstractMatrix = [0 0]; equal_axes::Bool = true, point_size::Real = 10, plot_leading_vector::Bool = false, scale::Real=0.15)
    fig = GLM.Figure();
    ax = GLM.Axis(fig[1,1], title = "", aspect = GLM.DataAspect());

    if P != [0 0]
        GLM.scatter!(ax,P[1,:], P[2,:], markersize=point_size, color = :black); # plots tracking points
    end

    for t in traj
        GLM.lines!(ax,map(e->GLM.Point2f(e[1:2]),t.u),color=:blue);
        plot_leading_vector && GLM.lines!(ax,map(e->GLM.Point2f(e[1:2]+[sin(x),cos(x)]),t.u),color=:red);
        GLM.poly!(ax, getPlanarShapes(t.u[1],scale), color = GLM.RGBA{Float32}(0, 0, 1, 0.1));
        GLM.poly!(ax, getPlanarShapes(t.u[end],scale), color = GLM.RGBA{Float32}(160/255, 32/255, 240/255, 0.2));
    end

    if equal_axes
        # what follows is a hack to give the plot a "square" shape
        GLM.reset_limits!(ax); # updates axes limits to fit the content of ax
        L = argmax(ax.finallimits[].widths); 
        M = ax.finallimits[].widths[L]/2;

        limits = Vector{Float64}(undef,4);

        for i = 1:2
            m = ax.finallimits[].origin[i] + ax.finallimits[].widths[i]/2;
            limits[2*i-1:2*i] = [m-M, m+M];
        end

        GLM.limits!(ax,limits...);
    end

    return fig;
end

function plotCameraTrajectoriesIn3D(traj::Vector{VS.VSTrajectory{VS.planar}}, P::AbstractMatrix = [0 0]; point_size::Real = 10, endpoints_size::Real = 25)
    fig = GLM.Figure();
    ax = GLM.Axis3(fig[1,1], title = "", aspect = :data);

    if P != [0 0]
        GLM.scatter!(ax, P[1,:], P[2,:], zeros(size(P,2)), markersize = point_size, color = :black); # plots tracking points
    end

    for t in traj
        GLM.lines!(ax, map(GLM.Point3f,t.u));
        GLM.scatter!(ax, map(GLM.Point3f,[t.u[1],t.u[end]]), color = [:green,:red], markersize = endpoints_size, marker = [:hexagon,'x']);
    end

    return fig;
end

function getPlanarShapes(state::Vector{<:Real},size::Real = 0.15)
    x, y, θ = state[1:3];

    basic_shape = size*[GLM.Point2f(0,-1), GLM.Point2f(0.75,1), GLM.Point2f(-0.75,1)]; # vertices of a triangle which is "looking up"
    R = [cos(θ) sin(θ); -sin(θ) cos(θ)]; # matrix that allows to rotate the triangle in the same direction of the camera
    
    camera_shape = Vector{GLM.Point2f}(undef,3);

    for i = 1:3
        camera_shape[i] = R*basic_shape[i] + [x;y]; # roto-translation of each vertex to the correct position
    end

    return camera_shape;
end