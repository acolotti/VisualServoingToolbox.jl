# VisualServoingToolbox

VisualServoingToolbox.jl is a Julia package to study the behavior and equilibria of Image-Based Visual Servoing (IBVS) systems. It allows to run simulations of both ideal and realistic visual servoing systems in a reliable way. The package implements several state-of-the-art simulation techniques for dynamical systems evolving on SE(3) (i.e., the group of roto-translations in 3D), ranging from simple adaptations of the Euler method, useful for simulating real-world behavior, to advanced techniques for the simulation of stiff systems, which are needed to capture the system’s behavior in pathological situations (e.g., if the camera tries to hit the object, as it is the case in two of the videos shown below).

the package also provides a fast, numerical alternative for the equilibria computation methodology developed in [this paper](https://hal.science/hal-04628273v1), based on homotopy continuation, as well as a simple interface to produce publication-ready plots and videos.

Here are a few examples of animations that can be done with it:

https://github.com/acolotti/VisualServoingToolbox.jl/assets/119665057/7e50b125-03d1-4f95-984a-cbe53a5e2dc9

https://github.com/acolotti/VisualServoingToolbox.jl/assets/119665057/cbd19430-6536-49b8-9472-a04f4a88d8c0

https://github.com/acolotti/VisualServoingToolbox.jl/assets/119665057/5ec0f7e7-b42f-41df-8479-dd8d5265a3aa

One of the aims of the package is also to provide an easy interface to extend it with new, custom systems, even though it is currently undocumented.

## Headless plotting

GLMakie remains the default plotting backend, so existing code can continue to
load the package as usual:

```julia
using VisualServoingToolbox
```

For a headless environment, activate CairoMakie before loading the toolbox:

```julia
using CairoMakie
using VisualServoingToolbox
```

For a numerical-only process that does not need a plotting backend, disable the
default GLMakie activation before loading the package:

```julia
ENV["VST_AUTOSTART_GLMAKIE"] = "false"
using VisualServoingToolbox
```

Plotting functions can still be used after a Makie backend is activated later.

If a workflow only needs system definitions, trajectory types, utilities, and
plotting, it can also skip the differential-equation simulator and equilibria
solver. This is an opt-in compile-time preference; the default remains the full
package for backward compatibility:

```julia
using Preferences
set_preferences!("VisualServoingToolbox", "lightweight" => true)
```

Restart Julia after changing this preference. Set it back to `false` to restore
the full package. In lightweight mode, `simulate` and the equilibria-computation
functions are intentionally unavailable.

VisualServoingToolbox.jl has been developed mainly for internal use and it will most likely remain in permanent beta. It is released under the [CRAPL](https://matt.might.net/articles/crapl/).
