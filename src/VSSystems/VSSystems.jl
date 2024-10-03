module VSSystems

    import ForwardDiff
    import LinearAlgebra
    import HDF5

    import ..RotationConversions as RC

    struct VSTrajectory{T}
        u::Vector{Vector{Float64}}
        t::Vector{Float64}
    end

    include("VSSystems_planar.jl")
    include("VSSystems_circular.jl")
    include("VSSystems_spatial_dp.jl")
    include("VSSystems_spatial.jl")
    include("VSSystems_controllers.jl")

    export LevenbergMarquardtController, pseudoInverseController, transposeController, desiredPosePseudoInverseController, metaController, # controllers
        circular, planar, spatial, spatialdesiredpose, # systems
        getEquilibriumIndex, getError, getPotentialHessian, getReducedHessian, getTrajectoriesErrors, getVelocities, # utilities
        VSTrajectory # datastructures

    function getPotentialHessian(f::Function,x::Vector{<:Real},p)
        vf(x) = (dx=zeros(eltype(x),length(x));f(dx,x,p,0);return dx);
        #println(vf(x));
        jacobian(x) = ForwardDiff.jacobian(vf,x);
    
        return jacobian(x);
    end

    function getTrajectoriesErrors(trs::Vector{VSTrajectory{T}},p,returnNorm::Bool = true) where {T}
        n = length(trs);
        e = Array{AbstractVecOrMat}(undef,n);
        d = length(trs[1].u[1]);

        for i = 1:n
            nTimes = length(trs[i].u); 
            q = zeros(d,nTimes);
            for j = 1:nTimes
                q[:,j] = trs[i].u[j];
            end
            e[i] = getError(T,q,p,returnNorm);
        end
    
        return e;
    end

    function getTrackingPointsDepthFromTrajectories(trs::Vector{VSTrajectory{T}},P::AbstractMatrix) where {T}
        n = length(trs);
        Z = Vector{AbstractVecOrMat}(undef,n);
        d = length(trs[1].u[1]);
    
        for i = 1:n
            nTimes = length(trs[i].u); 
            q = zeros(d,nTimes);
            for j = 1:nTimes
                q[:,j] = trs[i].u[j];
            end
            Z[i] = getTrackingPointsDepth(T,q,P);
        end
    
        return Z;
    end

# end module
end
