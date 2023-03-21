function planarL(x,P)
    type = typeof(x[1]);
    N = size(P,2);
    mL = zeros(type,N,3);
    R = [cos(x[3]) -sin(x[3]); sin(x[3]) cos(x[3])];
    Pc = R*(P.-x[1:2]);
    for i = 1:N
        mL[i,:] = [-Pc[2,i] Pc[1,i] -(Pc[1,i]^2 + Pc[2,i]^2)]./(Pc[2,i]^2);
    end

    return mL;
end

function planars(x,P)
    R = [cos(x[3]) -sin(x[3]); sin(x[3]) cos(x[3])];
    Pc = R*(P.-x[1:2]);
    return (Pc[1,:]./Pc[2,:]);
end

function planar(dx, x, p, t)
    λ = 1;
    P = p[1];
    sd = p[2];

    N = size(P,2);
    L = zeros(eltype(x),N,3);

    R = [cos(x[3]) -sin(x[3]); sin(x[3]) cos(x[3])];
    Pc = R*(P.-x[1:2]);

    s = Pc[1,:]./Pc[2,:];
    for i = 1:N
        L[i,:] = [-Pc[2,i] Pc[1,i] -(Pc[1,i]^2 + Pc[2,i]^2)]./(Pc[2,i]^2);
    end

    dtmp = -λ*transpose(L)*(s[:] - sd[:]);
    #dtmp = -λ*LinearAlgebra.inv(transpose(L)*L)*transpose(L)*(s[:] - sd[:]);
    
    dx[1:2] = transpose(R)*dtmp[1:2];
    dx[3] = dtmp[3];
    
    return nothing
end

function getError(::typeof(planar),x::AbstractVecOrMat,p::AbstractMatrix,returnNorm::Bool = true)
    n = size(x,2);
    P = p[1:2,:];
    sd = p[3,:];
    N = size(P,2);
    if returnNorm
        e = zeros(eltype(x),n,1);
        for i = 1:n 
            e[i] = sum((a - b)^2 for (a,b) in zip(planars(x[:,i],P),sd));
        end
        return e;
    else
        e = zeros(eltype(x),n,N);
        for i = 1:n 
            e[i,:] = planars(x[:,i],P) - sd;
        end
        return e;
    end
end

function getTrackingPointsDepth(::typeof(planar),X::AbstractVecOrMat,P::AbstractMatrix)
    n = size(X,2);
    N = size(P,2);
    Z = zeros(N,n);
    
    R(θ) = [cos(θ) -sin(θ); sin(θ) cos(θ)];

    for i = 1:n
        Z[:,i] = transpose((R(X[3,i])*(P .- X[1:2,i]))[2,:]);
    end

    return Z;
end