module EquilibriaComputation

import ..RotationConversions as RC
import ..VSSystems as VS
import ..Utilities as Uti
import ..Simulator as Sim
import ..Desiderata as Des
import HomotopyContinuation as HC
import LinearAlgebra as LA
import NLsolve as NL

export TaskParameters, computePerfectApproximationEquilibria, computeDesiredPoseApproximationEquilibria, fastDesiredPoseApproximationEquilibria, improveEquilibriaPrecision, reconstructValidPoses, saveTaskParametersToFile, loadTaskParametersFromFile

#==
    Structure containing all we need about a given visual servoing task.
    The idea is that a task is a given configuration of points with (potentially) several desired poses.
==#
struct TaskParameters
    P::Matrix{Float64}
    sds::Vector{Matrix{Float64}}
    oxcs::Vector{Vector{Float64}}
    name::String
end

#==
    Functions to compute all equilibria of a given task using HomotopyContinuation.
    WARNING: perfect approximation and second order only work with 4 points!
==#

function computePerfectApproximationEquilibria(ex::TaskParameters; mask::Vector{<:Integer} = [range(1,length(ex.sds))...])
    function distMat(P)
        d = zeros(6);
        k = 1;
    
        norm²(u) = sum(a^2 for a in u);
    
        for i = 1:3
            for j = i+1:4
                d[k] = norm²(P[:,i]-P[:,j]);
                k += 1;
            end
        end
    
        return d;
    end

    function spatialField(sx,sy,Z,P,sd)
        N = size(P,2);
        vf = Array{HC.Expression}(undef,6);
        
        vf[1] = sum((sd[1,i]-sx[i])*prod(Z[1:end.!=i]) for i=1:N)
        vf[2] = sum((sd[2,i]-sy[i])*prod(Z[1:end.!=i]) for i=1:N)
        vf[3] = sum((sx[i]^2 - sx[i]*sd[1,i] + sy[i]^2 - sy[i]*sd[2,i])*prod(Z[1:end.!=i]) for i=1:N);
        vf[4] = sum((sx[i]*sy[i]*(sx[i]-sd[1,i]) + (1+sy[i]^2)*(sy[i]-sd[2,i])) for i=1:N);
        vf[5] = sum((sx[i]*sy[i]*(sy[i]-sd[2,i]) + (1+sx[i]^2)*(sx[i]-sd[1,i])) for i=1:N);
        vf[6] = sum((sx[i]*sd[2,i] - sy[i]*sd[1,i]) for i=1:N);
    
        return vf;
    end

    mm = map(e->(1<=e<=length(ex.sds)),mask); # creates a vector of bools, "true" means legal mask value
    mask = mask[mm]; # filter mask
    
    if isempty(mask)
        @error "Mask is either empty or only contains illegal values.";
    elseif !all(mm) # there is at least one "false" in mm
        @warn "Detected illegal values in mask.";
    end

    HC.@var sx[1:4],sy[1:4],Z[1:4];

    P = ex.P;
    sds = ex.sds[mask];

    if size(P,2) != 4
        @error "Error: the equilibria computation only works for the 4 points case.";
    end

    d = distMat(P);
    
    constraints = [-d[1] + Z[1]^2 + sx[1]^2*Z[1]^2 + sy[1]^2*Z[1]^2 - 2*Z[1]*Z[2] - 2*sx[1]*sx[2]*Z[1]*Z[2] - 
     2*sy[1]*sy[2]*Z[1]*Z[2] + Z[2]^2 + sx[2]^2*Z[2]^2 + sy[2]^2*Z[2]^2,
     -d[2] + Z[1]^2 + sx[1]^2*Z[1]^2 + sy[1]^2*Z[1]^2 - 2*Z[1]*Z[3] - 2*sx[1]*sx[3]*Z[1]*Z[3] - 
     2*sy[1]*sy[3]*Z[1]*Z[3] + Z[3]^2 + sx[3]^2*Z[3]^2 + sy[3]^2*Z[3]^2,
     -d[3] + Z[1]^2 + sx[1]^2*Z[1]^2 + sy[1]^2*Z[1]^2 - 2*Z[1]*Z[4] - 2*sx[1]*sx[4]*Z[1]*Z[4] - 
        2*sy[1]*sy[4]*Z[1]*Z[4] + Z[4]^2 + sx[4]^2*Z[4]^2 + sy[4]^2*Z[4]^2,
     -d[4] + Z[2]^2 + sx[2]^2*Z[2]^2 + sy[2]^2*Z[2]^2 - 2*Z[2]*Z[3] - 2*sx[2]*sx[3]*Z[2]*Z[3] - 
        2*sy[2]*sy[3]*Z[2]*Z[3] + Z[3]^2 + sx[3]^2*Z[3]^2 + sy[3]^2*Z[3]^2,
     -d[5] + Z[2]^2 + sx[2]^2*Z[2]^2 + sy[2]^2*Z[2]^2 - 2*Z[2]*Z[4] - 2*sx[2]*sx[4]*Z[2]*Z[4] - 
        2*sy[2]*sy[4]*Z[2]*Z[4] + Z[4]^2 + sx[4]^2*Z[4]^2 + sy[4]^2*Z[4]^2,
     -d[6] + Z[3]^2 + sx[3]^2*Z[3]^2 + sy[3]^2*Z[3]^2 - 2*Z[3]*Z[4] - 2*sx[3]*sx[4]*Z[3]*Z[4] - 
        2*sy[3]*sy[4]*Z[3]*Z[4] + Z[4]^2 + sx[4]^2*Z[4]^2 + sy[4]^2*Z[4]^2];
    
    equilibria = Array{Any}(undef,length(sds));

    for i in eachindex(sds)
        f = HC.System([spatialField(sx,sy,Z,P,sds[i]);constraints],variables=[sx...,sy...,Z...]);
        res = HC.solve(f);
        equilibria[i] = HC.real_solutions(res);
    end

    return equilibria;
end


function computeSecondOrderEquilibria(ex::TaskParameters; mask::Vector{<:Integer} = [range(1,length(ex.sds))...])
    function distMat(P)
        d = zeros(6);
        k = 1;
    
        norm²(u) = sum(a^2 for a in u);
    
        for i = 1:3
            for j = i+1:4
                d[k] = norm²(P[:,i]-P[:,j]);
                k += 1;
            end
        end
    
        return d;
    end

    function spatialField(sx,sy,Z,P,sd,Zd)
        N = size(P,2);
        vf = Array{HC.Expression}(undef,6);
        
        vf[1] = sum((sd[1,i]-sx[i])*(Z[i]+Zd[i])*(prod(Z[1:end.!=i])*prod(Zd[1:end.!=i])) for i=1:N)
        vf[2] = sum((sd[2,i]-sy[i])*(Z[i]+Zd[i])*(prod(Z[1:end.!=i])*prod(Zd[1:end.!=i])) for i=1:N)
        vf[3] = sum(( (Z[i]*sd[1,i] + Zd[i]*sx[i])*(sx[i]-sd[1,i]) + (Z[i]*sd[2,i] + Zd[i]*sy[i])*(sy[i]-sd[2,i]) )*(prod(Z[1:end.!=i])*prod(Zd[1:end.!=i])) for i=1:N);
        vf[4] = sum((sx[i]*sy[i]*(sx[i]-sd[1,i]) + (1+sy[i]^2)*(sy[i]-sd[2,i]))+(sd[1,i]*sd[2,i]*(sx[i]-sd[1,i]) + (1+sd[2,i]^2)*(sy[i]-sd[2,i])) for i=1:N);
        vf[5] = sum((sx[i]*sy[i]*(sy[i]-sd[2,i]) + (1+sx[i]^2)*(sx[i]-sd[1,i]))+(sd[1,i]*sd[2,i]*(sy[i]-sd[2,i]) + (1+sd[1,i]^2)*(sx[i]-sd[1,i])) for i=1:N);
        vf[6] = sum((sx[i]*sd[2,i] - sy[i]*sd[1,i]) for i=1:N);

        return vf;
    end

    mm = map(e->(1<=e<=length(ex.sds)),mask); # creates a vector of bools, "true" means legal mask value
    mask = mask[mm]; # filter mask
    
    if isempty(mask)
        @error "Mask is either empty or only contains illegal values.";
    elseif !all(mm) # there is at least one "false" in mm
        @warn "Detected illegal values in mask.";
    end

    HC.@var sx[1:4],sy[1:4],Z[1:4];

    P = ex.P;
    sds = ex.sds[mask];

    if size(P,2) != 4
        @error "Error: the equilibria computation only works for the 4 points case.";
    end

    Zd = Vector{Vector{Float64}}(undef,length(mask));

    for (i,dp) in enumerate(ex.oxcs[mask])
        Zd[i] = (RC.quaternionToRotationMatrix(dp[4:end])'*(P .- dp[1:3]))[3,:];
    end

    d = distMat(P);
    
    constraints = [-d[1] + Z[1]^2 + sx[1]^2*Z[1]^2 + sy[1]^2*Z[1]^2 - 2*Z[1]*Z[2] - 2*sx[1]*sx[2]*Z[1]*Z[2] - 
     2*sy[1]*sy[2]*Z[1]*Z[2] + Z[2]^2 + sx[2]^2*Z[2]^2 + sy[2]^2*Z[2]^2,
     -d[2] + Z[1]^2 + sx[1]^2*Z[1]^2 + sy[1]^2*Z[1]^2 - 2*Z[1]*Z[3] - 2*sx[1]*sx[3]*Z[1]*Z[3] - 
     2*sy[1]*sy[3]*Z[1]*Z[3] + Z[3]^2 + sx[3]^2*Z[3]^2 + sy[3]^2*Z[3]^2,
     -d[3] + Z[1]^2 + sx[1]^2*Z[1]^2 + sy[1]^2*Z[1]^2 - 2*Z[1]*Z[4] - 2*sx[1]*sx[4]*Z[1]*Z[4] - 
        2*sy[1]*sy[4]*Z[1]*Z[4] + Z[4]^2 + sx[4]^2*Z[4]^2 + sy[4]^2*Z[4]^2,
     -d[4] + Z[2]^2 + sx[2]^2*Z[2]^2 + sy[2]^2*Z[2]^2 - 2*Z[2]*Z[3] - 2*sx[2]*sx[3]*Z[2]*Z[3] - 
        2*sy[2]*sy[3]*Z[2]*Z[3] + Z[3]^2 + sx[3]^2*Z[3]^2 + sy[3]^2*Z[3]^2,
     -d[5] + Z[2]^2 + sx[2]^2*Z[2]^2 + sy[2]^2*Z[2]^2 - 2*Z[2]*Z[4] - 2*sx[2]*sx[4]*Z[2]*Z[4] - 
        2*sy[2]*sy[4]*Z[2]*Z[4] + Z[4]^2 + sx[4]^2*Z[4]^2 + sy[4]^2*Z[4]^2,
     -d[6] + Z[3]^2 + sx[3]^2*Z[3]^2 + sy[3]^2*Z[3]^2 - 2*Z[3]*Z[4] - 2*sx[3]*sx[4]*Z[3]*Z[4] - 
        2*sy[3]*sy[4]*Z[3]*Z[4] + Z[4]^2 + sx[4]^2*Z[4]^2 + sy[4]^2*Z[4]^2];
    
    equilibria = Array{Any}(undef,length(sds));

    for i in eachindex(sds)
        f = HC.System([spatialField(sx,sy,Z,P,sds[i],Zd[i]);constraints],variables=[sx...,sy...,Z...]);
        res = HC.solve(f);
        equilibria[i] = HC.real_solutions(res);
    end

    return equilibria;
end

function computeDesiredPoseApproximationEquilibria(ex::TaskParameters; mask::Vector{<:Integer} = [range(1,length(ex.sds))...])
    function distanceMatrix(P)
        N = size(P,2);
        d = zeros(N,N);
    
        norm²(u) = sum(a^2 for a in u);
    
        for i = 1:N-1
            for j = i+1:N
                d[i,j] = norm²(P[:,i]-P[:,j]);
            end
        end
    
        return d;
    end

    function spatialDesiredPoseField(sx,sy,sd,Zd)
        N = length(sx);
        vf = Array{HC.Expression}(undef,6);
        
        vf[1] = sum((sd[1,i]-sx[i])*prod(Zd[1:end.!=i]) for i=1:N)
        vf[2] = sum((sd[2,i]-sy[i])*prod(Zd[1:end.!=i]) for i=1:N)
        vf[3] = sum((-sd[1,i]^2 + sx[i]*sd[1,i] - sd[2,i]^2 + sy[i]*sd[2,i])*prod(Zd[1:end.!=i]) for i=1:N);
        vf[4] = sum((sd[1,i]*sd[2,i]*(sx[i]-sd[1,i]) + (1+sd[2,i]^2)*(sy[i]-sd[2,i])) for i=1:N);
        vf[5] = sum((sd[1,i]*sd[2,i]*(sy[i]-sd[2,i]) + (1+sd[1,i]^2)*(sx[i]-sd[1,i])) for i=1:N);
        vf[6] = sum((sx[i]*sd[2,i] - sy[i]*sd[1,i]) for i=1:N);
    
        return vf;
    end

    mm = map(e->(1<=e<=length(ex.sds)),mask); # creates a vector of bools, "true" means legal mask value
    mask = mask[mm]; # filter mask
    
    if isempty(mask)
        @error "Mask is either empty or only contains illegal values.";
    elseif !all(mm) # there is at least one "false" in mm
        @warn "Detected illegal values in mask.";
    end

    P = ex.P;
    sds = ex.sds[mask];
    Zd = Vector{Vector{Float64}}(undef,length(mask));

    for (i,dp) in enumerate(ex.oxcs[mask])
        Zd[i] = (RC.quaternionToRotationMatrix(dp[4:end])'*(P .- dp[1:3]))[3,:];
    end

    N = size(P,2);

    HC.@var sx[1:N],sy[1:N],Z[1:N];

    d = distanceMatrix(P);
    
    constraints = [-d[i,j] + Z[i]^2 + sx[i]^2*Z[i]^2 + sy[i]^2*Z[i]^2 - 2*Z[i]*Z[j] - 2*sx[i]*sx[j]*Z[i]*Z[j] - 2*sy[i]*sy[j]*Z[i]*Z[j] + Z[j]^2 + sx[j]^2*Z[j]^2 + sy[j]^2*Z[j]^2 for i = 1:N-1 for j = i+1:N];
    
    equilibria = Array{Any}(undef,length(sds));

    for i in eachindex(sds)
        f = HC.System([spatialDesiredPoseField(sx,sy,sds[i],Zd[i]);constraints],variables=[sx...,sy...,Z...]);
        res = HC.solve(f);
        equilibria[i] = HC.real_solutions(res);
    end

    return equilibria;
end

function fastDesiredPoseApproximationEquilibria(ex::TaskParameters, mask::Vector{<:Integer} = [range(1,length(ex.sds))...])
    function distanceMatrix(P)
        N = size(P,2);
        d = zeros(N,N);
    
        norm²(u) = sum(a^2 for a in u);
    
        for i = 1:N-1
            for j = i+1:N
                d[i,j] = norm²(P[:,i]-P[:,j]);
            end
        end
    
        return d;
    end

    function getVariablesReduction(sd,Zd)
        N = size(sd,2);
        L = zeros(2*N,6);

        for i = 1:N
            L[2*i-1:2*i,:] = [-1/Zd[i] 0 sd[1,i]/Zd[i] sd[1,i]*sd[2,i] -(1+sd[1,i]^2) sd[2,i];
                            0 -1/Zd[i] sd[2,i]/Zd[i] 1+sd[2,i]^2 -sd[1,i]*sd[2,i] -sd[1,i]];
        end

        return LA.nullspace(L'), L*LA.pinv(L)*sd[:];
    end

    mm = map(e->(1<=e<=length(ex.sds)),mask); # creates a vector of bools, "true" means legal mask value
    mask = mask[mm]; # filter mask
    
    if isempty(mask)
        @error "Mask is either empty or only contains illegal values.";
    elseif !all(mm) # there is at least one "false" in mm
        @warn "Detected illegal values in mask.";
    end

    P = ex.P;
    sds = ex.sds[mask];
    N = size(P,2);
    Zd = Vector{Vector{Float64}}(undef,length(mask));

    for (i,dp) in enumerate(ex.oxcs[mask])
        Zd[i] = (RC.quaternionToRotationMatrix(dp[4:end])'*(P .- dp[1:3]))[3,:];
    end

    HC.@var λ[1:2*(N-3)],Z[1:N]; # sx_v = [sx_4,...,sx_N], sy_v = [sy_4,...,sy_N]

    d = distanceMatrix(P);
    
    equilibria = Array{Any}(undef,length(sds));

    for i in eachindex(sds)
        A,b = getVariablesReduction(sds[i],Zd[i]);

        s = b - A*λ; # s = [sx_1(λ),sy_1(λ),...,sx_N(λ),sy_N(λ)]

        sx = [s[i] for i in eachindex(s) if i % 2 == 1]; # sx = [sx_1(λ),..., sx_N(λ)]
        sy = [s[i] for i in eachindex(s) if i % 2 == 0]; # sy = [sy_1(λ),..., sy_N(λ)]

        constraints = [-d[i,j] + Z[i]^2 + sx[i]^2*Z[i]^2 + sy[i]^2*Z[i]^2 - 2*Z[i]*Z[j] - 2*sx[i]*sx[j]*Z[i]*Z[j] - 2*sy[i]*sy[j]*Z[i]*Z[j] + Z[j]^2 + sx[j]^2*Z[j]^2 + sy[j]^2*Z[j]^2 for i = 1:N-1 for j = i+1:N];

        f = HC.System(constraints,variables=[λ...,Z...]);
        res = HC.solve(f);

        r_sols = HC.real_solutions(res);

        equilibria[i] = Vector{Vector{Float64}}(undef, length(r_sols));

        for (j,e) in enumerate(r_sols)
            λ_sol = e[1:2*(N-3)];
            s_sol = b - A*λ_sol;

            sx_t = [s_sol[i] for i in eachindex(s_sol) if i % 2 == 1]; # sx = [sx_1(λ),..., sx_N(λ)]
            sy_t = [s_sol[i] for i in eachindex(s_sol) if i % 2 == 0]; # sy = [sy_1(λ),..., sy_N(λ)]

            equilibria[i][j] = [sx_t..., sy_t..., e[2*(N-3)+1:end]...];
        end
    end

    return equilibria;
end

function improveEquilibriaPrecision(ex::TaskParameters, poses::Vector{Vector{Vector{T}}} where T <: Real; controller = VS.transposeController, quiet::Bool = false)
    improved_poses = Vector{Vector{Vector{Float64}}}(undef,length(poses));
    
    for i in eachindex(improved_poses)
        quiet || println("Subcase $i analysis launched.");

        improved_poses[i] = Vector{Vector{Float64}}(undef, length(poses[i]));

        P = ex.P;
        sd = ex.sds[i];

        function spatialNLsolve!(dx,x)
            pᵣ = x[1:3]; # robot's position in world frame
            qᵣ = x[4:end]; # quaternion representing the robot's orientation as o^q_c (camera to world)

            N = size(P,2);
            L = zeros(eltype(x),2*N,6);

            oRc = RC.quaternionToRotationMatrix(qᵣ);
            Pc = transpose(oRc)*(P.-pᵣ);

            s = [transpose(Pc[1,:]./Pc[3,:]); transpose(Pc[2,:]./Pc[3,:])];

            for i = 1:N
                L[2*i-1:2*i,:] = [-1/Pc[3,i] 0 s[1,i]/Pc[3,i] s[1,i]*s[2,i] -(1+s[1,i]^2) s[2,i];
                                0 -1/Pc[3,i] s[2,i]/Pc[3,i] 1+s[2,i]^2 -s[1,i]*s[2,i] -s[1,i]];
            end
            
            dx[1:6] = controller(1,L,s,sd);
            dx[7] = sum(x[4:end].^2)-1;
        end
        
        spatialField(x) = (dx=Vector{Float64}(undef,7);VS.spatial(dx,x,[P,sd,controller],0.0);return dx);

        for s in eachindex(poses[i])
            quiet || println("Root finding $s launched..");
            init_time = time_ns();
            rts = NL.nlsolve(spatialNLsolve!,poses[i][s],method=:newton, autodiff=:forward, ftol = 1e-50)
            quiet || println("""
            Finished. Elapsed time: $((time_ns() - init_time)/(1e9)) seconds.

            State comparison:
                Before: $(poses[i][s])
                After: $(rts.zero)
            
            Vector field comparison:
                Before: $(spatialField(poses[i][s]))
                After: $(spatialField(rts.zero))

            Error comparison:
                Before: $(VS.getError(VS.spatial,poses[i][s],[P,sd,controller]))
                After: $(VS.getError(VS.spatial,rts.zero,[P,sd,controller]))
            
            """)
            
            improved_poses[i][s] = rts.zero;
        end
    end

    return improved_poses;
end

function reconstructValidPoses(equilibria, oP, coplanar::Bool; visibility_check::Bool = true, superpose_check::Bool = true) # "equilibria" is the output of "computeEquilibria"
    centroid(P) = sum(map(i->P[:,i],1:size(P,2)))./size(P,2);
    poses = Vector{Vector{Vector{Float64}}}(undef,length(equilibria));
    valid_equilibria = Vector{Vector{Vector{Float64}}}(undef,length(equilibria));

    N = size(oP,2);
    ok = centroid(oP);
    Q = oP .- ok;

    for (ex,s) in enumerate(equilibria)
        # points coordinates in camera-frame; filter out poses not respecting visibility
        println("\nConverting task $ex...\n")
        n = length(s);
        Pc = Vector{Matrix{Float64}}(undef,n);
        mask = Vector{Int64}(undef,n);
        cnt_visibility = 1;
        println("Number of solutions: $n");

        for i = 1:n
            sx = s[i][1:N];
            sy = s[i][N+1:2*N];
            Z = s[i][2*N+1:end];

            if superpose_check && any(abs(z)<1e-8 for z in Z)
                println("Solution $i appears to be superposed to a point.");
            elseif visibility_check && any(z<=-1e-8 for z in Z)
                println("Solution $i does not respect visibility.");
            else
                Pc[cnt_visibility] = zeros(3,N);
                Pc[cnt_visibility][1,:] = sx.*Z;
                Pc[cnt_visibility][2,:] = sy.*Z;
                Pc[cnt_visibility][3,:] = Z;

                mask[cnt_visibility] = i;

                cnt_visibility += 1;
            end
        end

        if cnt_visibility != n+1
            println("Number of acceptable solutions: $(cnt_visibility-1)")
        end

        Pc = Pc[1:cnt_visibility-1];
        mask = mask[1:cnt_visibility-1];

        # Pose reconstruction
        local_poses = Vector{Vector{Float64}}(undef, cnt_visibility-1);

        cnt_feasibility = 1;

        for i in eachindex(Pc)
            ck = centroid(Pc[i]);
            P = Pc[i].-ck;

            U,_,V = LA.svd(Q*P');
            Ω = coplanar ? U*LA.diagm([1,1,sign(LA.det(U*V'))])*V' : U*V';

            if LA.det(Ω) > 0
                local_poses[cnt_feasibility] = [(ok - Ω*ck)...,RC.rotationMatrixToQuaternion(Ω)...];
                mask[cnt_feasibility] = mask[i];
                cnt_feasibility += 1;
            else
                println("Solution $i is improper.")
            end
        end

        println("Number of feasible solutions: $(cnt_feasibility-1)");

        poses[ex] = copy(local_poses[1:cnt_feasibility-1]);
        valid_equilibria[ex] = copy(s[mask[1:cnt_feasibility-1]]);
    end

    return poses, valid_equilibria;
end

function saveTaskParametersToFile(ex::TaskParameters, poses::Vector{Vector{Vector{T}}} where T<:Real, valid_equilibria::Vector{Vector{Vector{T}}} where T<:Real)
    if !(length(ex.sds) == length(poses) == length(valid_equilibria))
        @error "The number of desired poses and solutions do not agree.";
    end

    for i in eachindex(ex.sds)
        Uti.saveEquilibriaAndPosesToFile("task_parameters/$(ex.name).$i.h5", valid_equilibria[i], poses[i], ex.P, ex.sds[i],ex.oxcs[i]);
    end
end

function loadTaskParametersFromFile(exName::String)
    files = readdir("task_parameters");
    files = files[[exName == f[1:findfirst('.',f)-1] for f in files]]; # keeps only the files related to the task
    
    if isempty(files)
        @error "No tasks named $exName found.";
    end

    n = length(files);
    P = 0;
    sds = Vector{Matrix{Float64}}(undef,n);
    oxcs = Vector{Vector{Float64}}(undef,n);
    poses = Vector{Vector{Vector{Float64}}}(undef,n);
    equilibria = Vector{Vector{Vector{Float64}}}(undef,n);

    for i in range(1,n)
        equilibria[i],poses[i],P,sds[i],oxcs[i] = Uti.loadEquilibriaAndPosesFromFile("task_parameters/$exName.$i.h5");
    end

    return TaskParameters(P,sds,oxcs,exName), equilibria, poses;
end

function simulateAroundEquilibrium(ex::TaskParameters, poses::Vector{Vector{Vector{T}}} where T<:Real, N::Int64 = 100; controllers = [VS.transposeController], mask::Vector{Vector{T}} where T<:Integer = [[0]], times::Vector{T} where T<:Real = [1e7])
    norm²(u) = sum(a^2 for a in u[1:3]);
    
    function shortestdistance(p)
        n = length(p);

        if n == 1
            return 1; # random number; having only one equilibrium is a very specific case
        else
            d = zeros(n*(n-1)÷2);
            k = 1;
        
            for i = 1:n-1
                for j = i+1:n
                    d[k] = norm²(p[i]-p[j]);
                    k += 1;
                end
            end
        
            return minimum(d);
        end
    end

    function closestpoint(p,eq)
        return findmin([norm²(p-e) for e in eq])[2];
    end

    function initialpoints(p,rad,P,N)
        area = [p[1:3].-rad p[1:3].+rad];
        Xt = Des.getRandomPointsFromArea(N,area);
        #return Des.getValidRandomPoints(VS.spatial,N,P,Xt,quiet=true)
        return [Xt; kron(ones(N)',p[4:end])];
    end
    
    n = length(ex.sds);
    m_poses = Vector{Vector{Vector{Float64}}}(undef, n);
    M = Vector{Vector{Matrix{Int64}}}(undef,n);

    if mask == [[0]]
        m_poses = poses;
    else
        for i in range(1,n)
            mm = map(e->(1<=e<=length(poses[i])),mask[i]);
            mask[i] = mask[i][mm];

            if isempty(mask[i])
                @error "Mask $i is either empty or only contains illegal values.";
            elseif !all(mm) # there is at least one "false" in mm
                @warn "Detected illegal values in mask $i.";
            end

            m_poses[i] = poses[i][mask[i]];
        end
    end

    for i in range(1,n)
        #rad = shortestdistance(poses[i])/100;
        rad = 0.01;
        M[i] = [zeros(Int64,length(poses[i]),length(poses[i])) for _ in eachindex(controllers)];

        println("Simulating pose $i.")

        for (j,p) in enumerate(m_poses[i])
            X = initialpoints(p,rad,ex.P,N);

            for c_index in eachindex(controllers)
                trs = Sim.simulate(VS.spatial,X,ex.P,ex.sds[i],times[c_index],controller=controllers[c_index],quiet=true);

                for traj in trs
                    M[i][c_index][j,closestpoint(traj.u[end],poses[i])] += 1;
                end
            end
        end
    end

    return M;
end

# module end
end
