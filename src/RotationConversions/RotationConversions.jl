module RotationConversions

import LinearAlgebra

export angleAxisToRotationMatrix, quaternionToRotationMatrix, angleAxisToQuaternion,
       rotationMatrixToQuaternion, quaternionToAngleAxis, rotationMatrixToAngleAxis,
       rotationMatrixToEulerAngles, EulerAnglesToRotationMatrix

function rotationMatrixToEulerAngles(R::AbstractMatrix)
    if abs(R[3,1]) - 1 <= 1e-30
        θ₁ = asin(R[3,1]);
        θ₂ = π - θ₁;

        ψ₁ = atan((R[3,2:3]/cos(θ₁))...);
        ϕ₁ = atan(([R[2,1],R[1,1]]/cos(θ₁))...);
        ψ₂ = atan((R[3,2:3]/cos(θ₂))...);
        ϕ₂ = atan(([R[2,1],R[1,1]]/cos(θ₂))...);

        if any(abs.([ψ₁,θ₁,ϕ₁]) .> π)
            return [ψ₂,θ₂,ϕ₂];
        else
            return [ψ₁,θ₁,ϕ₁];
        end
    else
        ϕ = 0;
        s = -sign(R[3,1]);
        θ = s*π/2;
        ψ = atan(s*R[1,2],s*R[1,3]);

        return [ψ,θ,ϕ];
    end
end

function EulerAnglesToRotationMatrix(A::AbstractVector)
    a,b,c = A;
    return [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)] * [cos(b) 0 sin(b); 0 1 0; -sin(b) 0 cos(b)] * [cos(c) -sin(c) 0; sin(c) cos(c) 0; 0 0 1];
end

function angleAxisToRotationMatrix(S::AbstractVector)
    K(u) = [0 -u[3] u[2]; u[3] 0 -u[1]; -u[2] u[1] 0];

    θ = LinearAlgebra.norm(S);
    
    if θ <= 1e-30
    	R = LinearAlgebra.diagm([1, 1, 1]);
    else
    	u = S/θ;
	R = LinearAlgebra.diagm([1, 1, 1]) + sin(θ)*K(u) + (1-cos(θ))*K(u)*K(u);
    end
    
    return R;
end

function quaternionToRotationMatrix(q::AbstractVector)
    s = 1/(sum(a^2 for a in q));
	return [1-2*s*(q[3]^2+q[4]^2) 2*s*(q[2]*q[3]-q[4]*q[1]) 2*s*(q[2]*q[4]+q[3]*q[1]);
			2*s*(q[2]*q[3]+q[4]*q[1]) 1-2*s*(q[2]^2+q[4]^2) 2*s*(q[3]*q[4]-q[2]*q[1]);
			2*s*(q[2]*q[4]-q[3]*q[1]) 2*s*(q[3]*q[4]+q[2]*q[1]) 1-2*s*(q[2]^2+q[3]^2)];
end

function angleAxisToQuaternion(S::AbstractVector)
    θ = LinearAlgebra.norm(S);

    if θ < 1e-30
        return [1.; 0.; 0.; 0.];
    else
        u = S/θ;
        return [cos(θ/2); u[1]*sin(θ/2); u[2]*sin(θ/2); u[3]*sin(θ/2)];
    end
end

function rotationMatrixToQuaternion(R::AbstractMatrix)
    S = rotationMatrixToAngleAxis(R);
    return angleAxisToQuaternion(S);
end

function quaternionToAngleAxis(q::AbstractVector)
    R = quaternionToRotationMatrix(q);
    return rotationMatrixToAngleAxis(R);
end

function rotationMatrixToAngleAxis(R::AbstractMatrix)
    cosθ = (LinearAlgebra.tr(R)-1)/2;
    if abs(cosθ) > 1
        @warn "Ill conditioned rotation matrix: cosθ = $cosθ."
        cosθ = sign(cosθ);
    end
    θ = acos(cosθ);
    _,V = LinearAlgebra.eigen(R);    
    ω = real(V[:,3]);

    norm²(v) = sum(c^2 for c in v[:]);
    R1 = angleAxisToRotationMatrix(ω.*θ);
    R2 = angleAxisToRotationMatrix(-ω.*θ);

    if norm²(R - R1) < norm²(R - R2)
        return ω.*θ;
    else
        return -ω.*θ;
    end
end

function quaternionConjugate(q::AbstractVector)
    return [q[1];-q[2:4]];
end

end
