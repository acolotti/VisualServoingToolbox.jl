ENV["VST_AUTOSTART_GLMAKIE"] = "false"

using Preferences
set_preferences!("VisualServoingToolbox", "lightweight" => true; force=true)

using VisualServoingToolbox
using Test

@testset "VisualServoingToolbox.jl" begin
    @test isdefined(VisualServoingToolbox, :plotCameraTrajectories)
    @test isdefined(VisualServoingToolbox, :VSTrajectory)
    @test ismissing(VisualServoingToolbox.Plotter.GLM.current_backend())
    @test !isdefined(VisualServoingToolbox, :Simulator)
end
