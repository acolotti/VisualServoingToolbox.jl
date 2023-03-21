function spatialdesiredpose(dx, x, p, t)
    λ = 1;
    P = p[1];
    sd = p[2];
    controller = p[3];
    Ld = p[4];

    pᵣ = x[1:3]; # robot's position in world frame
    qᵣ = x[4:end]; # quaternion representing the robot's orientation as o^q_c (camera to world)

    N = size(P,2);

    oRc = RC.quaternionToRotationMatrix(qᵣ);
    Pc = transpose(oRc)*(P.-pᵣ);

    s = [transpose(Pc[1,:]./Pc[3,:]); transpose(Pc[2,:]./Pc[3,:])];    

    v_c = controller(λ,Ld,s,sd);

    dx[1:3] = oRc*v_c[1:3];
    Ω(ω) = [0 -ω[1] -ω[2] -ω[3];
            ω[1] 0 ω[3] -ω[2];
            ω[2] -ω[3] 0 ω[1];
            ω[3] ω[2] -ω[1] 0];

    #W(q) = [-q[2] -q[3] -q[4]; q[1] -q[4] -q[3]; q[4] q[1] -q[2]; -q[3] q[2] q[1]];

    dx[4:7] = 0.5*Ω(v_c[4:end])*qᵣ;#0.5*W(qᵣ)*dtmp[4:end];
    
    return nothing
end

function getDesiredL(xd, P)
    N = size(P,2);
    L = zeros(eltype(xd),2*N,6);

    oRc = RC.quaternionToRotationMatrix(xd[4:end]);
    Pc = transpose(oRc)*(P.-xd[1:3]);

    s = [transpose(Pc[1,:]./Pc[3,:]); transpose(Pc[2,:]./Pc[3,:])];

    for i = 1:N
        L[2*i-1:2*i,:] = [-1/Pc[3,i] 0 s[1,i]/Pc[3,i] s[1,i]*s[2,i] -(1+s[1,i]^2) s[2,i];
                          0 -1/Pc[3,i] s[2,i]/Pc[3,i] 1+s[2,i]^2 -s[1,i]*s[2,i] -s[1,i]];
    end

    return L;
end