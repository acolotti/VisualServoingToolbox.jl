module Simulator

import DifferentialEquations as DE
import LinearAlgebra as LA

import ..VSSystems as VS

export simulate

"""
    simulate(f, x0, p, t; solver=TRBDF2, quiet=false)

Simulate the visual servoing system defined by `f`.

# Arguments
- `f::Function`: system's vector field.
- `x0::Vector{Vector{<:Real}}`: Vector of initial positions.
- `p`: system's parameters, depend on the chosen vector field.
- `sd::AbstractArray{<:Real}`: (f x N) matrix, where f is the features' dimension. Each column is a tracking point's wanted feature.
- `t::Real`: simulation time.
- `solver=DE.TRBDF2`: ODE solver. Can be choosen from: https://diffeq.sciml.ai/stable/solvers/ode_solve/
- `quiet::Bool=false`: option to suppress output.
"""
function simulate(f::Function, x0::Vector{Vector{T}} where T<:Real, p, t::Real; solver=DE.TRBDF2, quiet::Bool=false, 
    step_size::Union{Real,Nothing} = nothing, terminal_distance::Function = VS.squaredDistance, 
    terminal_state::Union{Vector{T} where T<:Real, Vector{Vector{T}} where T<:Real, Nothing} = nothing, terminal_threshold::Real = 1e-6)

    M0 = reduce(hcat,x0);

    return simulate(f,M0,p,t,solver=solver,quiet=quiet,step_size=step_size,terminal_distance=terminal_distance,terminal_state=terminal_state,terminal_threshold=terminal_threshold);
end
"""
    simulate(f, x0, p, t; solver=TRBDF2, quiet=false)

Simulate the visual servoing system defined by `f`.

# Arguments
- `f::Function`: system's vector field.
- `x0::AbstractArray{<:Real}`: (n x s) matrix, where n is the system's dimension. Each column is a different initial position.
- `p`: system's parameters, depend on the chosen vector field.
- `sd::AbstractArray{<:Real}`: (f x N) matrix, where f is the features' dimension. Each column is a tracking point's wanted feature.
- `t::Real`: simulation time.
- `solver=DE.TRBDF2`: ODE solver. Can be choosen from: https://diffeq.sciml.ai/stable/solvers/ode_solve/
- `quiet::Bool=false`: option to suppress output.
"""
function simulate(f::Function, x0::AbstractArray{<:Real}, p, t::Real; solver=DE.TRBDF2, quiet::Bool=false, 
    step_size::Union{Real,Nothing} = nothing, terminal_distance::Function = VS.squaredDistance, 
    terminal_state::Union{Vector{T} where T<:Real, Vector{Vector{T}} where T<:Real, Nothing} = nothing, terminal_threshold::Real = 1e-6)

    if !isa(p,NamedTuple)
        @error "Passing the simulation's parameters as a vector is deprecated, use named tuples instead.";
    end

    P = p.P;
    sd = p.sd;
    
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
            trajectories[i] = singleSimulation(f,x0[:,i],p,t,solver,step_size=step_size,
                terminal_distance = terminal_distance, terminal_state=terminal_state,terminal_threshold=terminal_threshold);
        end
    else
        for i = 1:s
            println("Simulation $i started..");
            trajectories[i] = singleSimulation(f,x0[:,i],p,t,solver,step_size=step_size,
                terminal_distance = terminal_distance, terminal_state=terminal_state,terminal_threshold=terminal_threshold);
            println("Finished.");
        end
    end

    return trajectories;
end

"""
    simulate(f, x0, P, sd, t; solver=TRBDF2, quiet=false)

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
    quiet::Bool=false, step_size::Union{Real,Nothing} = nothing, terminal_state::Union{Vector{T} where T<:Real, Vector{Vector{T}} where T<:Real, Nothing} = nothing, terminal_threshold::Real = 1e-6)

    return simulate(f,x0,(P = P,sd = sd,controller = controller),t,solver=solver,quiet=quiet,step_size=step_size,terminal_state=terminal_state,terminal_threshold=terminal_threshold);
end

function singleSimulation(f::Function, x0::AbstractArray{<:Real}, p::NamedTuple, t::Real, solver; step_size::Union{Real,Nothing} = nothing, 
    terminal_distance::Function = VS.squaredDistance, 
    terminal_state::Union{Vector{T} where T<:Real, Vector{Vector{T}} where T<:Real, Nothing} = nothing, terminal_threshold::Real = 1e-6)
    
    tol = 1e-8;

    prob = DE.ODEProblem(f,x0,(0.0, t),p);

    if isa(terminal_state,Nothing)
        sol = isa(step_size,Nothing) ? DE.solve(prob,solver(),reltol=tol,abstol=tol) : DE.solve(prob,DE.Euler(),dt=step_size);
    else
        condition(u,t,integrator) = isa(terminal_state, Vector{Vector{T}} where T<:Real) ? minimum([(terminal_distance(f,u,t_s)-terminal_threshold^2) for t_s in terminal_state]) : terminal_distance(f,u,terminal_state)-terminal_threshold^2;
        affect!(integrator) = DE.terminate!(integrator);
        cb = DE.ContinuousCallback(condition,affect!);
        sol = isa(step_size,Nothing) ? DE.solve(prob,solver(),callback=cb) : DE.solve(prob,DE.Euler(),callback=cb,dt = step_size);

        if sol.t[end] == t
            @warn "Warning: a simulation reached the final time without activating the termination condition.";
        end
    end

    
    return VS.VSTrajectory{f}(sol.u,sol.t);
end

end
