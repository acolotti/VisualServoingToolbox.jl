module VisualServoingToolbox
# Visual Servoing Toolbox for simulations, plotting, animations & equilibria computation
include("RotationConversions/RotationConversions.jl")
include("VSSystems/VSSystems.jl")
include("Simulator/Simulator.jl")
include("Utilities/Utilities.jl")
include("Desiderata/Desiderata.jl")
include("Plotter/Plotter.jl")
include("EquilibriaComputation/EquilibriaComputation.jl")

using .RotationConversions
using .VSSystems
using .Utilities
using .Simulator
using .Desiderata
using .Plotter
using .EquilibriaComputation

mods = [RotationConversions,VSSystems,Utilities,Simulator,Desiderata,Plotter,EquilibriaComputation];

for mod in mods
    for name in names(mod, all=true)
        if Base.isexported(mod, name)
            @eval export $(name)
        end
    end
end

#end module
end
