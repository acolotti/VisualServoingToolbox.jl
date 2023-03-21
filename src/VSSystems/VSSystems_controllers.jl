function transposeController(λ,L,s,sd)
    return -λ*transpose(L)*(s[:] - sd[:]);
end

function pseudoInverseController(λ,L,s,sd)
    return -λ*LinearAlgebra.inv(transpose(L)*L)*transpose(L)*(s[:] - sd[:]);
end

function LevenbergMarquardtController(λ,L,s,sd)
    μ = 1;
    return -λ*LinearAlgebra.inv(transpose(L)*L + μ*LinearAlgebra.I)*transpose(L)*(s[:] - sd[:]);
end