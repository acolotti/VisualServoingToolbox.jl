function getFeaturesFromPoints(::typeof(VS.planar), P::AbstractMatrix, qd::AbstractVector)
    R = [cos(qd[3]) -sin(qd[3]); sin(qd[3]) cos(qd[3])];
    Pc = R*(P.-qd[1:2]);
    return (Pc[1,:]./Pc[2,:]);
end

function getValidRandomPoints(::typeof(VS.planar),N::Int,P::AbstractMatrix,M::AbstractVecOrMat=[0];quiet::Bool=false)
    if M == [0]
        M = getRandomPointsOutsideP(N,P,quiet=quiet);
    end
    possibleθ = -π:(π/4):π;
    θs = zeros(1,N);
    R(θ) = [cos(θ) -sin(θ); sin(θ) cos(θ)];

    for i = 1:N 
        t = Random.shuffle(possibleθ);
        for θ in t
            if all((R(θ)*(P .- M[:,i]))[2,:] .> 0)
                θs[i] = θ;
                break
            end
        end
    end

    return [M;θs];
end

function validateRandomPoints(::typeof(VS.planar),X::AbstractVecOrMat,P::AbstractMatrix)
    N = size(X,2);
    v = Array{Bool}(undef,N);

    R(θ) = [cos(θ) -sin(θ); sin(θ) cos(θ)];

    for i = 1:N
        v[i] = all((R(X[3,i])*(P .- X[1:2,i]))[2,:] .> 0);
    end

    return v;
end