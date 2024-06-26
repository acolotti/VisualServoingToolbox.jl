function spatial(dx, x, p, t)
    
    v_c = p.controller(var"#self#",x,p);

    if haskey(p,:saturation) && LinearAlgebra.norm(v_c) > p.saturation
        v_c = (p.saturation * v_c)/LinearAlgebra.norm(v_c);
    end

    oRc = RC.quaternionToRotationMatrix(x[4:end]);

    dx[1:3] = oRc*v_c[1:3];
    Ω(ω) = [0 -ω[1] -ω[2] -ω[3];
            ω[1] 0 ω[3] -ω[2];
            ω[2] -ω[3] 0 ω[1];
            ω[3] ω[2] -ω[1] 0];

    #W(q) = [-q[2] -q[3] -q[4]; q[1] -q[4] -q[3]; q[4] q[1] -q[2]; -q[3] q[2] q[1]];

    dx[4:7] = 0.5*Ω(v_c[4:end])*x[4:end];
    
    return nothing
end

function cartesianCoordinatesInteractionMatrix(::typeof(spatial),s,Z)
    N = length(Z);
    L = zeros(eltype(s),2*N,6);

    for i = 1:N
        L[2*i-1:2*i,:] = [-1/Z[i] 0 s[1,i]/Z[i] s[1,i]*s[2,i] -(1+s[1,i]^2) s[2,i];
                          0 -1/Z[i] s[2,i]/Z[i] 1+s[2,i]^2 -s[1,i]*s[2,i] -s[1,i]];
    end

    return L;
end

function getError(::Union{typeof(spatial),typeof(spatialdesiredpose)},x::AbstractVecOrMat,p,returnNorm::Bool = true)
    n = size(x,2);
    P = p.P;
    sd = p.sd;
    N = size(P,2);

    R(qᵣ) = transpose(RC.quaternionToRotationMatrix(qᵣ)); #cRo

    s(x,P) = (Pc = R(x[4:end])*(P.-x[1:3]); return [transpose(Pc[1,:]./Pc[3,:]); transpose(Pc[2,:]./Pc[3,:])]);

    if returnNorm
        e = zeros(eltype(x),n);
        for i = 1:n 
            e[i] = 0.5*sum((a - b)^2 for (a,b) in zip(s(x[:,i],P)[:],sd[:]));
        end
        return e;
    else
        e = zeros(eltype(x),n,2*N);
        for i = 1:n 
            e[i,:] = s(x[:,i],P)[:] - sd[:];
        end
        return e;
    end
end

function getTrackingPointsDepth(::Union{typeof(spatial),typeof(spatialdesiredpose)},X::AbstractVecOrMat,P::AbstractMatrix)
    n = size(X,2);
    N = size(P,2);
    Z = zeros(N,n);
    
    R(qᵣ) = transpose(RC.quaternionToRotationMatrix(qᵣ)); #cRo

    for i = 1:n
        Z[:,i] = transpose((R(X[4:end,i])*(P .- X[1:3,i]))[3,:]);
    end

    return Z;
end

function getReducedHessian(f::Union{typeof(spatial),typeof(spatialdesiredpose)},x::Vector{<:Real},p,returnZ::Bool = false)
    J = getPotentialHessian(f,x,p);
    id = Matrix{Int}(LinearAlgebra.I,3,3);
    v = Matrix{Float64}(undef,1,4);
    for i = 1:4
        v[i] = x[3+i];
    end
    Z = [id zeros(3,3); zeros(4,3) LinearAlgebra.nullspace(v)]
    
    #Z = LinearAlgebra.nullspace([0 0 0 x[4:end]...]);
    
    if returnZ
        return (transpose(Z)*J*Z,Z);
    else
        return transpose(Z)*J*Z;
    end
end

function getEquilibriumIndex(f::Union{typeof(spatial),typeof(spatialdesiredpose)},x::Vector{<:Real},p)
    J = getReducedHessian(f,x,p);
    e,_ = LinearAlgebra.eigen(J);

    return sum((real(eig) > 0) ? 1 : 0 for eig in e);
end

function getDistanceFromHeteroclinicOrbit(trs)
    rid = HDF5.h5open("simulations/spatial_heteroclinic_N4.h5","r");
    het = (HDF5.read(rid["trs1"]))["u"];
    HDF5.close(rid);

    #println(het);

    e = Vector{Vector{Float64}}(undef,length(trs));

    for (i,traj) in enumerate(trs)
        e[i] = zeros(length(traj.u));

        for (j,p) in enumerate(traj.u)
            pstar = [p[1:3];-p[4:end]];            
            e[i][j] = minimum(sum(abs2,[het.-p het.-pstar],dims=1));
        end
    end

    return e;

end

function getVelocities(trs::Union{Vector{VSTrajectory{spatial}},Vector{VSTrajectory{spatialdesiredpose}}}, P::AbstractMatrix{<:Real}, sd::AbstractArray{<:Real}; controller = transposeController)
    return getVelocities(trs,[P,sd,controller]);
end

function getVelocities(trs::Union{Vector{VSTrajectory{spatial}},Vector{VSTrajectory{spatialdesiredpose}}}, p::AbstractVector)
    type(::Vector{VSTrajectory{T}}) where T = T;
    f = type(trs);

    # the formulation above is kept for readability, but an equivalent formulation would be:
    # f = eltype(trs).parameters[1];

    tau = Vector{Vector{Vector{Float64}}}(undef,length(trs));
    W(q) = [-q[2] -q[3] -q[4]; q[1] -q[4] -q[3]; q[4] q[1] -q[2]; -q[3] q[2] q[1]];
    field(x) = (dx = zeros(7); f(dx,x,p,0); return [dx[1:3]...,2*LinearAlgebra.pinv(W(x[4:end]))*dx[4:end]...]);

    for (i,traj) in enumerate(trs)
        tau[i] = [field(x) for x in traj.u];
    end

    return tau;
end

function squaredDistance(::Union{typeof(spatial),typeof(spatialdesiredpose)},u::Vector{<:Real},v::Vector{<:Real})
    norm²(v) = sum(e^2 for e in v);
    qnorm_u = u[4:end]./LinearAlgebra.norm(u[4:end]);
    qnorm_v = v[4:end]./LinearAlgebra.norm(v[4:end]);

    return norm²(u[1:3]-v[1:3]) + min(norm²(qnorm_u - qnorm_v),norm²(qnorm_u + qnorm_v));
end