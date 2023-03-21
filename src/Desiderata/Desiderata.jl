module Desiderata

import Distributions
import HDF5
import Random
import LinearAlgebra as LA

import ..RotationConversions as RC
import ..VSSystems as VS

include("Desiderata_planar.jl")
include("Desiderata_circular.jl")
include("Desiderata_spatial.jl")

export getPointsAndFeatures, getRandomPointsFromArea, getRandomPointsOutsideP, getPointsAndFeaturesFromFile,
        savePointsAndFeaturesToFile, getValidRandomPoints

export circleSampling, getRandomPointsFromArea, getRandomPointsOutsideP, getValidRandomPoints

const systemsInfo = Dict(VS.planar => (2, [0; -sqrt(2); 0]), VS.circular => (2, [0; 0]), VS.spatial => (3, [0; -sqrt(2); 0; 1; 0; 0; 0]));

function noSuperposition(s::AbstractArray)
    n = size(s,1);
    for i=1:n-1
        for j=i+1:n
            if s[i,:] == s[j,:]
                return false;
            end
        end
    end
    return true;
end

#= TO BE CHANGED
+ currently relies on systemsInfo
+ name should be more explicit: getRandomPointsAndWantedFeatures

function getPointsAndFeatures(f::Function, N::Int; qd = 0)    
    if qd == 0
        (n,qd) = systemsInfo[f];
    else
        (n,) = systemsInfo[f];
    end

    P = rand(1:20,n,N);
    sd = getFeaturesFromPoints(f,P,qd);

    while !noSuperposition(sd)
        P = rand(1:20,n,N);
        sd = getFeaturesFromPoints(f,P,qd);
    end

    return P,sd;
end=#

function getRandomPointsFromArea(::Function,N::Int, area::AbstractMatrix)
    return getRandomPointsFromArea(N,area);
end

function getRandomPointsFromArea(N::Int, area::AbstractMatrix)
    # N    : number of points to be produced
    # area : (n x 2) matrix, where the first column is the LB of the box
    #        and the second column is the UB of the box

    n = size(area,1);
    M = zeros(Float64,n,N);

    for i=1:n 
        M[i,:] = rand(Distributions.Uniform(area[i,1],area[i,2]),1,N);
    end

    return M;
end

function getRandomPointsOutsideP(::Function,N::Int, P::AbstractMatrix; quiet::Bool = false)
    return getRandomPointsOutsideP(N,P,quiet=quiet);
end

function getRandomPointsOutsideP(N::Int, P::AbstractMatrix; quiet::Bool = false)
    (l,u) = getEnclosingRectangle(P);
    r = maximum(u - l);
    n = length(l);
    M = zeros(Float64,n,N);   
    j = 1;
    tries = 0;

    while j <= N
        for i = 1:n
            M[i,j] = rand(Distributions.Uniform(l[i]-r,u[i]+r));
        end
        if !all(l .< M[:,j] .< u)
            j += 1;
        else
            tries += 1;
        end
    end

    if !quiet && tries > 0
        @warn "Failed tries: $tries."
    end

    return M;

end

function circleSampling(center::Vector{T} where T<:Real, axes::Vector{Vector{T}} where T<:Real, radius::Real, N::Integer)
    θs = range(0, 2*pi, length = N+1);

    if LA.rank([axes[1] axes[2]]) < 2
        @error """
        Axes coincide:
            #1: $(axes[1])
            #2: $(axes[2])
        It is not possible to individuate the circle's hyper-plan.
        """;
    end 

    normalize(v) = v./LA.norm(v);

    normal_axes = [axes[1], normalize(axes[2] - ((axes[1]'*axes[2])/(LA.norm(axes[1])^2)) .* axes[1])];
    return [center+radius.*(normal_axes[1].*cos(θ) + normal_axes[2].*sin(θ)) for θ in θs[1:end-1]];
end

function getEnclosingRectangle(P::AbstractMatrix)
    n = size(P,1);
    l = zeros(n);
    u = zeros(n);

    for i in axes(P,1)
        l[i] = minimum(P[i,:]);
        u[i] = maximum(P[i,:]);
    end

    return l,u;
end

#=
TO BE CHANGED: they should follow "getPointsAndFeatures" restyle
function savePointsAndFeaturesToFile(filename::String, P::AbstractMatrix, sd::AbstractVecOrMat)
    fid = HDF5.h5open(filename,"w");
    
    fid["P"] = P;
    fid["sd"] = sd;
    
    close(fid);
end

function getPointsAndFeaturesFromFile(filename::String)
    rid = HDF5.h5open(filename,"r");
    P = read(rid["P"]);
    sd = read(rid["sd"]);

    close(rid);

    return P, sd;
end =#

# end module
end