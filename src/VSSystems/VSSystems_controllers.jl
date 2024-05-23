# Generic controllers, eventually they will become deprecated

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

function rotationOnlyController(λ,L,s,sd)
    Lr = L[:,4:end];
    τ = -λ*LinearAlgebra.inv(transpose(Lr)*Lr)*transpose(Lr)*(s[:] - sd[:]);
    return [0,0,0,τ...];
end

# Spatial-specific controllers

function pseudoInverseController(::typeof(spatial), x, p)
    P = p.P;
    sd = p.sd;
    λ = haskey(p,:λ) ? p.λ : 1;

    N = size(P,2);

    features_perturbation = (haskey(p,:features_noise_level) && p.features_noise_level != 0) ? rand(Dst.Uniform(-p.features_noise_level,p.features_noise_level),2,N) : zeros(2,N);
    depths_perturbation = (haskey(p,:depths_noise_level) && p.depths_noise_level != 0) ? rand(Dst.Uniform(-p.depths_noise_level,p.depths_noise_level),N) : zeros(N);

    oRc = RC.quaternionToRotationMatrix(x[4:end]);
    Pc = transpose(oRc)*(P.-x[1:3]);

    s = [transpose(Pc[1,:]./Pc[3,:]); transpose(Pc[2,:]./Pc[3,:])] + features_perturbation;
    Z = Pc[3,:] + depths_perturbation;

    L = cartesianCoordinatesInteractionMatrix(spatial, s, Z);

    return -λ*LinearAlgebra.inv(transpose(L)*L)*transpose(L)*(s[:] - sd[:]);
end

function transposeController(::typeof(spatial), x, p)
    P = p.P;
    sd = p.sd;
    λ = haskey(p,:λ) ? p.λ : 1;

    N = size(P,2);

    features_perturbation = (haskey(p,:features_noise_level) && p.features_noise_level != 0) ? rand(Dst.Uniform(-p.features_noise_level,p.features_noise_level),2,N) : zeros(2,N);
    depths_perturbation = (haskey(p,:depths_noise_level) && p.depths_noise_level != 0) ? rand(Dst.Uniform(-p.depths_noise_level,p.depths_noise_level),N) : zeros(N);

    oRc = RC.quaternionToRotationMatrix(x[4:end]);
    Pc = transpose(oRc)*(P.-x[1:3]);

    s = [transpose(Pc[1,:]./Pc[3,:]); transpose(Pc[2,:]./Pc[3,:])] + features_perturbation;
    Z = Pc[3,:] + depths_perturbation;

    L = cartesianCoordinatesInteractionMatrix(spatial, s, Z);

    return -λ*transpose(L)*(s[:] - sd[:]);
end

function LevenbergMarquardtController(::typeof(spatial), x, p)
    P = p.P;
    sd = p.sd;
    λ = haskey(p,:λ) ? p.λ : 1;

    N = size(P,2);

    features_perturbation = (haskey(p,:features_noise_level) && p.features_noise_level != 0) ? rand(Dst.Uniform(-p.features_noise_level,p.features_noise_level),2,N) : zeros(2,N);
    depths_perturbation = (haskey(p,:depths_noise_level) && p.depths_noise_level != 0) ? rand(Dst.Uniform(-p.depths_noise_level,p.depths_noise_level),N) : zeros(N);

    oRc = RC.quaternionToRotationMatrix(x[4:end]);
    Pc = transpose(oRc)*(P.-x[1:3]);

    s = [transpose(Pc[1,:]./Pc[3,:]); transpose(Pc[2,:]./Pc[3,:])] + features_perturbation;
    Z = Pc[3,:] + depths_perturbation;

    L = cartesianCoordinatesInteractionMatrix(spatial, s, Z);

    μ = 1;
    return -λ*LinearAlgebra.inv(transpose(L)*L + μ*LinearAlgebra.I)*transpose(L)*(s[:] - sd[:]);
end

function rotationOnlyController(::typeof(spatial), x, p)
    P = p.P;
    sd = p.sd;
    λ = haskey(p,:λ) ? p.λ : 1;

    N = size(P,2);

    features_perturbation = (haskey(p,:features_noise_level) && p.features_noise_level != 0) ? rand(Dst.Uniform(-p.features_noise_level,p.features_noise_level),2,N) : zeros(2,N);
    depths_perturbation = (haskey(p,:depths_noise_level) && p.depths_noise_level != 0) ? rand(Dst.Uniform(-p.depths_noise_level,p.depths_noise_level),N) : zeros(N);

    oRc = RC.quaternionToRotationMatrix(x[4:end]);
    Pc = transpose(oRc)*(P.-x[1:3]);

    s = [transpose(Pc[1,:]./Pc[3,:]); transpose(Pc[2,:]./Pc[3,:])] + features_perturbation;
    Z = Pc[3,:] + depths_perturbation;

    L = cartesianCoordinatesInteractionMatrix(spatial, s, Z)[:,4:end];

    τ = -λ*LinearAlgebra.inv(transpose(L)*L)*transpose(L)*(s[:] - sd[:]);

    return [0,0,0,τ...];
end

function MalisController(::typeof(spatial), x, p)
    P = p.P;
    sd = p.sd;
    λ = haskey(p,:λ) ? p.λ : 1;

    N = size(P,2);

    features_perturbation = (haskey(p,:features_noise_level) && p.features_noise_level != 0) ? rand(Dst.Uniform(-p.features_noise_level,p.features_noise_level),2,N) : zeros(2,N);
    depths_perturbation = (haskey(p,:depths_noise_level) && p.depths_noise_level != 0) ? rand(Dst.Uniform(-p.depths_noise_level,p.depths_noise_level),N) : zeros(N);

    oRc = RC.quaternionToRotationMatrix(x[4:end]);
    Pc = transpose(oRc)*(P.-x[1:3]);

    s = [transpose(Pc[1,:]./Pc[3,:]); transpose(Pc[2,:]./Pc[3,:])] + features_perturbation;
    Z = Pc[3,:] + depths_perturbation;

    L = cartesianCoordinatesInteractionMatrix(spatial, s, Z);

    if haskey(p,:Ld)
        Ld = p.Ld;
    elseif haskey(p,:Zd)
        Ld = cartesianCoordinatesInteractionMatrix(spatial, sd, p.Zd);
    elseif haskey(p,:xd)
        Rd = RC.quaternionToRotationMatrix(p.xd[4:end]);
        Zd = (transpose(Rd)*(P .- p.xd[1:3]))[3,:];
        Ld = cartesianCoordinatesInteractionMatrix(spatial, sd, Zd);
    else
        @error "Can't compute desired interaction matrix."
    end

    M = L + Ld;
    return -2*λ*LinearAlgebra.inv(transpose(M)*M)*transpose(M)*(s[:] - sd[:]);
end

function improvedMalisController(::typeof(spatial), x, p)
    P = p.P;
    sd = p.sd;
    λ = haskey(p,:λ) ? p.λ : 1;

    N = size(P,2);

    features_perturbation = (haskey(p,:features_noise_level) && p.features_noise_level != 0) ? rand(Dst.Uniform(-p.features_noise_level,p.features_noise_level),2,N) : zeros(2,N);
    depths_perturbation = (haskey(p,:depths_noise_level) && p.depths_noise_level != 0) ? rand(Dst.Uniform(-p.depths_noise_level,p.depths_noise_level),N) : zeros(N);

    oRc = RC.quaternionToRotationMatrix(x[4:end]);
    Pc = transpose(oRc)*(P.-x[1:3]);

    s = [transpose(Pc[1,:]./Pc[3,:]); transpose(Pc[2,:]./Pc[3,:])] + features_perturbation;
    Z = Pc[3,:] + depths_perturbation;

    L = cartesianCoordinatesInteractionMatrix(spatial, s, Z);

    if haskey(p,:xd)
        otd = p.xd[1:3];
        oRd = RC.quaternionToRotationMatrix(p.xd[4:end]);
        Zd = (transpose(oRd)*(P .- otd[1:3]))[3,:];
        Ld = cartesianCoordinatesInteractionMatrix(spatial, sd, Zd);

        cRd = transpose(oRc)*oRd;
        ctd = transpose(oRc)*(otd - x[1:3]);

        T = [cRd [0 -ctd[3] ctd[2]; ctd[3] 0 -ctd[1]; -ctd[2] ctd[1] 0]; zeros(3,3) cRd]
    else
        @error "Desired pose is required to compute the desired interaction matrix and the adjoint transformation."
    end

    M = L + Ld*LinearAlgebra.inv(T);
    return -2*λ*LinearAlgebra.inv(transpose(M)*M)*transpose(M)*(s[:] - sd[:]);
end