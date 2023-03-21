function circular(dx, x, p, t)
    λ = 1;
    P = p[1];
    sd = p[2];

    N = size(P,2);
    L = zeros(eltype(x),2*N,2);

    pseudo_inverse = false;
    #colNorm(A,d=1) = sqrt.(sum(abs2,A,dims=d));

    Pc = P.-x;
    normsPc = sqrt.(sum(abs2,Pc,dims=1));
    s = Pc./normsPc;

    for i = 1:N
        L[2*i-1:2*i,:] = [(Pc[1,i]^2/normsPc[i]^3 - 1/normsPc[i]) Pc[1,i]*Pc[2,i]/normsPc[i]^3; Pc[1,i]*Pc[2,i]/normsPc[i]^3 (Pc[2,i]^2/normsPc[i]^3-1/normsPc[i])];
    end
    if pseudo_inverse
        dx[1:2] = -λ*LinearAlgebra.inv(transpose(L)*L)*transpose(L)*(s[:] - sd[:]);
    else
        dx[1:2] = -λ*transpose(L)*(s[:] - sd[:]);
    end

    return nothing
end

function getError(::typeof(circular),x::AbstractVecOrMat,p::AbstractMatrix,returnNorm::Bool = true)
    n = size(x,2);
    P = p[1:2,:];
    sd = p[3:4,:];
    N = size(P,2);

    s(x,P) = (P.-x)./(sqrt.(sum(abs2,P.-x,dims=1)));

    if returnNorm
        e = zeros(eltype(x),n,1);
        for i = 1:n 
            e[i] = sum((a - b)^2 for (a,b) in zip(s(x[:,i],P)[:],sd[:]));
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