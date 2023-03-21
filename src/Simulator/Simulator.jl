module Simulator

import DifferentialEquations as DE
import LinearAlgebra as LA

import ..VSSystems as VS

export simulate

"""
    simulate(f, x0, p, t; controller=VS.transposeController, solver=TRBDF2, quiet=false)

Simulate the visual servoing system defined by `f`.

# Arguments
- `f::Function`: system's vector field.
- `x0::AbstractArray{<:Real}`: (n x s) matrix, where n is the system's dimension. Each column is a different initial position.
- `p::Vector{Any}`: system's parameters, depend on the chosen vector field.
- `sd::AbstractArray{<:Real}`: (f x N) matrix, where f is the features' dimension. Each column is a tracking point's wanted feature.
- `t::Real`: simulation time.
- `solver=DE.TRBDF2`: ODE solver. Can be choosen from: https://diffeq.sciml.ai/stable/solvers/ode_solve/
- `quiet::Bool=false`: option to suppress output.
"""
function simulate(f::Function, x0::AbstractArray{<:Real}, p::Vector{Any}, t::Real; solver=DE.TRBDF2, quiet::Bool=false, 
    step_size::Union{Real,Nothing} = nothing, terminal_state::Union{Vector{T} where T<:Real, Nothing} = nothing, terminal_threshold::Real = 1e-6)
    
    P = p[1];
    sd = p[2];
    
    if size(P,2) != size(sd,2)
        if size(P,2) == size(sd,1)
            sd = transpose(sd);
            @warn "Columns mismatch; transpose(sd) used instead.";
        else
            @error "Number of columns of P and sd must match.";
        end
    end

    s = size(x0,2);
    trajectories = Array{VS.VSTrajectory{f}}(undef,s);

    if quiet
        for i = 1:s
            trajectories[i] = singleSimulation(f,x0[:,i],p,t,solver,step_size=step_size,terminal_state=terminal_state,terminal_threshold=terminal_threshold);
        end
    else
        for i = 1:s
            println("Simulation $i started..");
            trajectories[i] = singleSimulation(f,x0[:,i],p,t,solver,step_size=step_size,terminal_state=terminal_state,terminal_threshold=terminal_threshold);
            println("Finished.");
        end
    end

    return trajectories;
end

"""
    simulate(f, x0, P, sd, t; controller=VS.transposeController, solver=TRBDF2, quiet=false)

Simulate the visual servoing system defined by `f`, function kept for back-compatibility.

# Arguments
- `f::Function`: system's vector field.
- `x0::AbstractArray{<:Real}`: (n x s) matrix, where n is the system's dimension. Each column is a different initial position.
- `P::AbstractMatrix{<:Real}`: (d x N) matrix, where d is the dimension of the Euclidean space. Each column is a tracking point's position.
- `sd::AbstractArray{<:Real}`: (f x N) matrix, where f is the features' dimension. Each column is a tracking point's wanted feature.
- `t::Real`: simulation time.
- `controller=VS.transposeController`: VS controller to be used. Can be choosen from "VSSystems_controllers".
- `solver=DE.TRBDF2`: ODE solver. Can be choosen from: https://diffeq.sciml.ai/stable/solvers/ode_solve/
- `quiet::Bool=false`: option to suppress output.
"""
function simulate(f::Function, x0::AbstractArray{<:Real}, P::AbstractMatrix{<:Real}, sd::AbstractArray{<:Real}, t::Real; controller=VS.transposeController, solver=DE.TRBDF2, 
    quiet::Bool=false, step_size::Union{Real,Nothing} = nothing, terminal_state::Union{Vector{T} where T<:Real, Nothing} = nothing, terminal_threshold::Real = 1e-6)

    return simulate(f,x0,[P,sd,controller],t,solver=solver,quiet=quiet,step_size=step_size,terminal_state=terminal_state,terminal_threshold=terminal_threshold);
end

function singleSimulation(f::Function, x0::AbstractArray{<:Real}, p::Vector{Any}, t::Real, solver; step_size::Union{Real,Nothing} = nothing, 
    terminal_state::Union{Vector{T} where T<:Real, Nothing} = nothing, terminal_threshold::Real = 1e-6)
    
    #p = [P, sd, controller];
    tol = 1e-8;

    prob = DE.ODEProblem(f,x0,(0.0, t),p);

    if isa(terminal_state,Nothing)
        sol = isa(step_size,Nothing) ? DE.solve(prob,solver(),reltol=tol,abstol=tol) : DE.solve(prob,DE.RK4(),dt=step_size);
    else
        norm²(v) = sum(v.^2);

        condition(u,t,integrator) = (qnorm = LA.norm(u[4:end]);return (norm²(u[1:3]-terminal_state[1:3]) + min(norm²(u[4:end]./qnorm - terminal_state[4:end]),norm²(u[4:end]./qnorm + terminal_state[4:end]))-terminal_threshold^2));
        affect!(integrator) = DE.terminate!(integrator);
        cb = DE.ContinuousCallback(condition,affect!);
        sol = isa(step_size,Nothing) ? DE.solve(prob,solver(),callback=cb) : DE.solve(prob,DE.RK4(),callback=cb,dt = step_size);

        if sol.t[end] == t
            @warn "Warning: a simulation reached the final time without activating the termination condition.";
        end
    end

    
    return VS.VSTrajectory{f}(sol.u,sol.t);
end

end