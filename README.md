# VisualServoingToolbox

VisualServoingToolbox.jl is a Julia package to study the behavior and equilibria of Image-Based Visual Servoing (IBVS) systems. It allows to run simulations of both ideal and realistic visual servoing systems in a reliable way. The package implements several state-of-the-art simulation techniques for dynamical systems evolving on SE(3) (i.e., the group of roto-translations in 3D), ranging from simple adaptations of the Euler method, useful for simulating real-world behavior, to advanced techniques for the simulation of stiff systems, which are needed to capture the system’s behavior in pathological situations (e.g., if the camera tries to hit the object, as it is the case in two of the videos shown below).

the package also provides a fast, numerical alternative for the equilibria computation methodology developed in [this paper](https://hal.science/hal-04628273v1), based on homotopy continuation, as well as a simple interface to produce publication-ready plots and videos.

Here are a few examples of animations that can be done with it:

https://github.com/acolotti/VisualServoingToolbox.jl/assets/119665057/7e50b125-03d1-4f95-984a-cbe53a5e2dc9

https://github.com/acolotti/VisualServoingToolbox.jl/assets/119665057/cbd19430-6536-49b8-9472-a04f4a88d8c0

https://github.com/acolotti/VisualServoingToolbox.jl/assets/119665057/5ec0f7e7-b42f-41df-8479-dd8d5265a3aa

One of the aims of the package is also to provide an easy interface to extend it with new, custom systems, even though it is currently undocumented.

VisualServoingToolbox.jl has been developed mainly for internal use and it will most likely remain in permanent beta. It is released under the [CRAPL](https://matt.might.net/articles/crapl/).
