module VisualServoingToolbox
# Visual Servoing Toolbox for simulations, plotting, animations & equilibria computation
using Preferences

const _LIGHTWEIGHT = @load_preference("lightweight", false)

include("RotationConversions/RotationConversions.jl")
include("VSSystems/VSSystems.jl")

if !_LIGHTWEIGHT
    include("Simulator/Simulator.jl")
end

include("Utilities/Utilities.jl")
include("Desiderata/Desiderata.jl")
include("Plotter/Plotter.jl")

if !_LIGHTWEIGHT
    include("EquilibriaComputation/EquilibriaComputation.jl")
end

using .RotationConversions
using .VSSystems

if !_LIGHTWEIGHT
    @eval using .Simulator
end

using .Utilities
using .Desiderata
using .Plotter

mods = [RotationConversions,VSSystems];

if !_LIGHTWEIGHT
    push!(mods, Simulator)
end

append!(mods, [Utilities,Desiderata,Plotter])

if !_LIGHTWEIGHT
    @eval using .EquilibriaComputation
    push!(mods, EquilibriaComputation)
end

for mod in mods
    for name in names(mod, all=true)
        if Base.isexported(mod, name)
            @eval export $(name)
        end
    end
end

function __init__()
    Plotter.activateDefaultBackend!()
end

#end module
end
