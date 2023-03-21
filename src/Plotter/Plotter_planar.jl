function plotCameraTrajectories(traj::Vector{VS.VSTrajectory{VS.planar}}, P::AbstractMatrix = [0 0]; scale::Real=0.15)

    (xTraj,zTraj,θTraj,xBegin,zBegin,θBegin,xFinal,zFinal,θFinal) = Uti.vectorizeTrajectories(traj);

    plt = Plots.plot(xTraj,zTraj,linecolor=:blue,aspect_ratio = 1,legend=:none);
    Plots.plot!(plt,xTraj + map(x->map(sin,x),θTraj),zTraj + map(x->map(cos,x),θTraj),linecolor=:red);

    if P != [0 0]
        Plots.plot!(plt,P[1,:], P[2,:], m=(10,:o), markercolor=:black,linetype=:scatter);
    end

    Plots.plot!(plt,getPlanarShapes(xBegin,zBegin,θBegin,scale),seriescolor=:purple,opacity=.1);
    Plots.plot!(plt,getPlanarShapes(xFinal,zFinal,θFinal,scale),seriescolor=:blue,opacity=.2);

    return plt;
end

function plotCameraTrajectoriesIn3D(traj::Vector{VS.VSTrajectory{VS.planar}}, P::AbstractMatrix = [0 0])

    (xTraj,zTraj,θTraj,xBegin,zBegin,θBegin,xFinal,zFinal,θFinal) = Uti.vectorizeTrajectories(traj);

    # if you want to choose your own color palette (i.e. you have a vector cols of 
    # colors) you can add the option palette=palette(cols)
    # https://docs.juliaplots.org/latest/generated/colorschemes/

    plt = Plots.plot3d(xTraj,zTraj,θTraj,legend=:none);

    if P != [0 0]
        Plots.plot3d!(plt,P[1,:], P[2,:], zeros(size(P,2)), m=(10,:o), markercolor=:black,linetype=:scatter);
    end
    Plots.plot3d!(plt,xBegin,zBegin,θBegin,m=(10,:hexagon),markercolor=:green,linetype=:scatter,opacity=.2);
    Plots.plot3d!(plt,xFinal,zFinal,θFinal,m=(10,:xcross),markercolor=:red,linetype=:scatter,opacity=.4);

    return plt;

end

function getPlanarShapes(x::Vector{<:Real},z::Vector{<:Real},θ::Vector{<:Real},mSize::Real = 0.15)
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