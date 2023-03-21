module Utilities

import HDF5

import ..VSSystems as VS

export loadEquilibriaAndPosesFromFile, saveEquilibriaAndPosesToFile,
loadEquilibriaFromFile, saveEquilibriaToFile,
loadTrajectoriesFromFile, saveTrajectoriesToFile,
vectorizeTrajectories

function vectorizeTrajectories(trs::Vector{VS.VSTrajectory{T}}) where {T}
    b = trs[1].u[1]; # initial position of the first simulation
    n = length(trs); # number of simulations
    q = length(b); # dimension of the state

    traj = [Vector{Vector{eltype(b)}}(undef,n) for i=1:q]; # for each element of the state, initialize a vector of vectors
                                                           # that will contain all the n (scalar) trajectories for that element 
    beg = [Vector{eltype(b)}(undef,n) for i=1:q];
    fin = [Vector{eltype(b)}(undef,n) for i=1:q];

    for i = 1:n
        nTime = length(trs[i].t); # time length of trajectory i
        tmpM = zeros(q,nTime); # row k contains the (scalar) trajectory of element j of the state
        for j = 1:nTime
            tmpM[:,j] = trs[i].u[j]
        end

        for j= 1:q
            traj[j][i] = tmpM[j,:];
            beg[j][i] = tmpM[j,1];
            fin[j][i] = tmpM[j,end];
        end
    end

    return traj...,beg...,fin...;
end

function saveTrajectoriesToFile(filename::String,trs::Vector{VS.VSTrajectory{T}}) where {T}
    n = length(trs);
    state_dim = length(trs[1].u[1]);

    fid = HDF5.h5open(filename,"w");

    fid["type"] = repr(T);

    for i = 1:n
        nt = length(trs[i].u);
        HDF5.create_group(fid,"trs$i");
        Mt = zeros(state_dim,nt);

        for j = 1:nt 
            Mt[:,j] = trs[i].u[j];
        end

        fid["trs$i"]["u"] = Mt;
        fid["trs$i"]["t"] = trs[i].t;
    end

    close(fid);
end

function loadTrajectoriesFromFile(filename::String)
    rid = HDF5.h5open(filename,"r");
    n = length(rid);

    sType = split(read(rid["type"]),".")[end];

    sysType = getfield(VS,Symbol(sType));

    trs = Array{VS.VSTrajectory{sysType}}(undef,n-1);

    for index in 1:n-1
        T = read(rid["trs$index"]);
        M = T["u"];
        nt = size(M,2);
        u = Vector{Vector{Float64}}(undef,nt);
        for i = 1:nt 
            u[i] = M[:,i];
        end
        trs[index] = VS.VSTrajectory{sysType}(u,T["t"]);
    end

    HDF5.close(rid);

    return trs;
end

function saveEquilibriaToFile(filename::String, equilibria::Vector{Vector{T}} where T<:Real,
                              oP::AbstractMatrix,sd::AbstractMatrix,oqc::AbstractVector)
    n = length(equilibria);
    fid = HDF5.h5open(filename,"w");

    HDF5.create_group(fid,"settings");
    fid["settings"]["oP"] = oP;
    fid["settings"]["sd"] = sd;
    fid["settings"]["oqc"] = oqc;

    HDF5.create_group(fid,"equilibria");
    for i = 1:n
        fid["equilibria"]["e$i"] = equilibria[i];
    end

    close(fid);
end

function saveEquilibriaAndPosesToFile(filename::String, equilibria::Vector{Vector{T}} where T<:Real, poses::Vector{Vector{T}} where T<:Real,
    oP::AbstractMatrix,sd::AbstractMatrix,oqc::AbstractVector)
n = length(equilibria);
fid = HDF5.h5open(filename,"w");

HDF5.create_group(fid,"settings");
fid["settings"]["oP"] = oP;
fid["settings"]["sd"] = sd;
fid["settings"]["oqc"] = oqc;

HDF5.create_group(fid,"equilibria");
for i = 1:n
    fid["equilibria"]["e$i"] = equilibria[i];
    fid["equilibria"]["p$i"] = poses[i];
end

close(fid);
end

function loadEquilibriaFromFile(filename::String)
    rid = HDF5.h5open(filename,"r");
    oP = 0;
    sd = 0;
    oqc = 0;

    if HDF5.haskey(rid,"settings")
        oP = read(rid["settings"]["oP"]);
        sd = read(rid["settings"]["sd"]);
        oqc = read(rid["settings"]["oqc"]);
    else
        @warn "Settings not saved in file. Should be updated ASAP!"
    end

    eq = Vector{Vector{Float64}}(undef,length(rid["equilibria"]));

    for (i,e) in enumerate(rid["equilibria"])
        eq[i] = read(e);
    end

    return (eq,oP,sd,oqc);
end

function loadEquilibriaAndPosesFromFile(filename::String)
    rid = HDF5.h5open(filename,"r");
    oP = 0;
    sd = 0;
    oqc = 0;

    if HDF5.haskey(rid,"settings")
        oP = read(rid["settings"]["oP"]);
        sd = read(rid["settings"]["sd"]);
        oqc = read(rid["settings"]["oqc"]);
    else
        @warn "Settings not saved in file. Should be updated ASAP!"
    end

    n = length(rid["equilibria"])÷2;
    eq = Vector{Vector{Float64}}(undef,n);
    poses = Vector{Vector{Float64}}(undef,n);

    for i in range(1,n)
        eq[i] = read(rid["equilibria"]["e$i"]);
        poses[i] = read(rid["equilibria"]["p$i"]);
    end

    return (eq,poses,oP,sd,oqc);
end

# module end
end