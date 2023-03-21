module VisualServoingToolbox
# Visual Servoing Toolbox for simulations, plotting & animations (equilibria computation coming soon!)
include("RotationConversions/RotationConversions.jl")
include("VSSystems/VSSystems.jl")
include("Simulator/Simulator.jl")
include("Utilities/Utilities.jl")
include("Desiderata/Desiderata.jl")
include("Plotter/Plotter.jl")

using .RotationConversions
using .VSSystems
using .Utilities
using .Simulator
using .Desiderata
using .Plotter

mods = [RotationConversions,VSSystems,Utilities,Simulator,Desiderata,Plotter];

for mod in mods
    for name in names(mod, all=true)
        if Base.isexported(mod, name)
            @eval export $(name)
        end
    end
end

#end module
end
