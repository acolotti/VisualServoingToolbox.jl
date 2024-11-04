function getFeaturesFromPoints(::typeof(VS.spatial), P::AbstractMatrix, qd::AbstractVector)
    oRc = RC.quaternionToRotationMatrix(qd[4:end]);
    Pc = transpose(oRc)*(P.-qd[1:3]);

    return [transpose(Pc[1,:]./Pc[3,:]); transpose(Pc[2,:]./Pc[3,:])];
end

function getUniformSampleFrom3Sphere()
    # all the stuff happening here is to pick a sample from a uniform distribution over a sphere.
    # for a reference, check: Marsaglia1972
    qt = ones(4);
    norm²(v) = sum(e^2 for e in v);
	
    while norm²(qt[1:2])>=1 || norm²(qt[3:4])>=1
        qt[1:4] = (rand(Float64,4).*2).-1;
    end

    K = sqrt((1-norm²(qt[1:2]))/norm²(qt[3:4]));
    qt[3:4] = qt[3:4].*K;

    return qt;
end

function getValidRandomPoints(::typeof(VS.spatial),N::Int,P::AbstractMatrix,M::AbstractVecOrMat=[0];quiet::Bool=false)
    if M == [0]
        M = getRandomPointsOutsideP(N,P,quiet=quiet);
    end

    if !quiet
        println("Positions: ",M);
    end
    
    R(qᵣ) = transpose(RC.quaternionToRotationMatrix(qᵣ));
    θs = zeros(4,N);
    norm²(v) = sum(e^2 for e in v);

    for i = 1:N 
        qt = ones(4);
        #println("Starting retrieval $i out of $N..")
        while true
            qt = getUniformSampleFrom3Sphere();
            
            !all((R(qt)*(P .- M[:,i]))[3,:] .> 0) || break
        end
        #println("Finished.")
        θs[:,i] = qt;
    end

    return [M;θs];
end

function getRandomOrientationInNeighborhood()
end

function validateRandomPoints(::typeof(VS.spatial),X::AbstractVecOrMat,P::AbstractMatrix)
    N = size(X,2);
    v = Array{Bool}(undef,N);

    R(qᵣ) = transpose(RC.quaternionToRotationMatrix(qᵣ));

    for i = 1:N
        v[i] = all((R(X[4:end,i])*(P .- X[1:3,i]))[3,:] .> 0);
    end

    return v;
end
