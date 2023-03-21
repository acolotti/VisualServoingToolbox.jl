function getFeaturesFromPoints(::typeof(VS.circular), P::AbstractMatrix, qd::AbstractVector)
    return (P.-qd)./(sqrt.(sum(abs2,P.-qd,dims=1)));
end

function getValidRandomPoints(::typeof(VS.circular),N::Int,P::AbstractMatrix;quiet::Bool=false)
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
        if all(M[:,j] != P[:,k] for k in axes(P,2))
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