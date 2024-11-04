"""
    [DEPRECATED]
    function plotCameraTrajectories(traj::Union{Vector{VS.VSTrajectory{VS.spatial}},Vector{VS.VSTrajectory{VS.spatialdesiredpose}}}, P::AbstractMatrix = [0 0]; axes_limits::Vector{T} where T<:Real = [0], colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, color_gradient::Bool = false, desired_pose::Vector{T} where T <: Real = [0], final_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, init_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, lines_order::Union{Nothing,Vector{T} where T<:Integer} = nothing, lines_thickness::Real = 2, point_size::Real = 20, scale::Real = 8, show_cameras::Bool = true, show_initial_cameras::Bool = true, style::Union{Symbol,Vector{T}} where T<:Real = :solid, theme::String = "", trajectories_limits::Vector{T} where T<:Real = [0])

    Plot the 3D trajectories represented by `traj`.

    # Arguments
    - `traj::Union{Vector{VS.VSTrajectory{VS.spatial}},Vector{VS.VSTrajectory{VS.spatialdesiredpose}}}`: trajectories to be plotted.
    - `P::AbstractMatrix`: matrix of tracking points (optional).

    # Keyword arguments
    - `axes_limits::Vector{T} where T<:Real`: 6-dimensional array to set the axes limits. Order is [x_min, x_max, y_min, y_max, z_min, z_max].
    - `colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol}`: trajectories' colors, can be either one color or a vector of colors. Default colors are the so-called Wong colors (colorblind-friendly)
    - `color_gradient::Bool`: changes the default colors to a gradient of colors.
    - `desired_pose::Vector{T} where T<:Real`: add the desired pose to the plot (as a green dotted camera).
    - `final_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol}`: final cameras' colors (same rules as `colors`).
    - `init_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol}`: initial cameras' colors (same rules as `colors`, only if `show_initial_cameras` is active).
    - `lines_order::Union{Nothing,Vector{T} where T<:Integer}`: changes the shape of the tracking points, from points to a 3D figure: it is a vector that defines in which order to write the lines. Example: the P matrix defines a square, lines_order might be [1,2,3,4,1], meaning that we draw lines connecting the first point to the second, the second to the third, etc., connecting back to the first one in the end to create a square.
    - `lines_thickness::Real`: thickness of the lines defined with lines_order
    - `point_size::Real`: tracking points size
    - `scale::Real`: scale of the cameras, as a percentage of the smallest axis' width. Example: scale = 8 means that the cameras will be around 8% of the smallest axis.
    - `show_cameras::Bool`: determines whether any camera is shown in the figure. 
    - `show_initial_cameras::Bool`: determines if the initial camera poses are shown (only if show_cameras is equal to true).
    - `style::Union{Symbol,Vector{T}} where T<:Real`: determines the linestyle of the trajectories, possible options are: :solid for solid lines, :dash for dashed ones, :dot for dotted one, etc.
    - `theme::String`: set plots theme, by default it's the GLMakie one. Other options are: "presentation" (for theme_ggplot2), "dark" (for theme_black)
    - `trajectories_limits::Vector{T} where T<:Real`: 6-dimensional array defining a box, the trajectories will be plotted only within the box. Order is [x_min, x_max, y_min, y_max, z_min, z_max]. Independent from axes_limits!
"""
function plotCameraTrajectories(traj::Union{Vector{VS.VSTrajectory{VS.spatial}},Vector{VS.VSTrajectory{VS.spatialdesiredpose}}}, P::AbstractMatrix; 
    axes_limits::Vector{T} where T<:Real = [0], colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, color_gradient::Bool = false, 
    desired_pose::Vector{T} where T <: Real = [0], final_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, 
    init_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, lines_order::Union{Nothing,Vector{T} where T<:Integer} = nothing, 
    lines_thickness::Real = 2, point_size::Real = 20, scale::Real = 8, show_cameras::Bool = true, show_initial_cameras::Bool = true, style::Union{Symbol,Vector{T}} where T<:Real = :solid, 
    theme::String = "", trajectories_limits::Vector{T} where T<:Real = [0])

    @warn "The passage of an explicit matrix of points P has been deprecated. A GeometricalObject should be provided instead."

    fig = GLM.Figure();
    ax = GLM.Axis3(fig[1,1], title = "", aspect = :data); # creates figure and axes

    if axes_limits != [0]
        GLM.limits!(ax,axes_limits...); # N.B. settings the limits manually prevents any subsequent, automatic change of limits performed by scatter, lines, etc.
    end

    O = Points(P);
    object_parameters = (lines_thickness = lines_thickness,lines_order = lines_order, point_size = point_size);

    plotCameraTrajectories!(ax,traj,O,
    color_gradient=color_gradient,colors=colors,desired_pose=desired_pose,final_colors=final_colors,init_colors=init_colors,object_parameters = object_parameters,
    scale=scale,show_cameras=show_cameras,show_initial_cameras=show_initial_cameras,style=style,theme=theme,trajectories_limits=trajectories_limits);

    return fig;
end

"""
    function plotCameraTrajectories(traj::Vector{VS.VSTrajectory{VS.spatial}}, O::Union{Nothing,GeometricalObject} = nothing; axes_limits::Vector{T} where T<:Real = [0], colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, color_gradient::Bool = false, desired_pose::Vector{T} where T <: Real = [0], final_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, init_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, object_parameters::NamedTuple = NamedTuple(), scale::Real = 8, show_cameras::Bool = true, show_initial_cameras::Bool = true, style::Union{Symbol,Vector{T}} where T<:Real = :solid, theme::String = "", trajectories_limits::Vector{T} where T<:Real = [0])

    Plot the 3D trajectories represented by `traj`.

    # Arguments
    - `traj::Vector{VS.VSTrajectory{VS.spatial}}`: trajectories to be plotted.
    - `O::GeometricalObject`: tracked 3D object (optional).

    # Keyword arguments
    - `axes_limits::Vector{T} where T<:Real`: 6-dimensional array to set the axes limits. Order is [x_min, x_max, y_min, y_max, z_min, z_max].
    - `colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol}`: trajectories' colors, can be either one color or a vector of colors. Default colors are the so-called Wong colors (colorblind-friendly)
    - `color_gradient::Bool`: changes the default colors to a gradient of colors.
    - `desired_pose::Vector{T} where T<:Real`: add the desired pose to the plot (as a green dotted camera).
    - `final_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol}`: final cameras' colors (same rules as `colors`).
    - `init_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol}`: initial cameras' colors (same rules as `colors`, only if `show_initial_cameras` is active).
    - `object_parameters::NamedTuple` : 3D object plotting parameters.
    - `scale::Real`: scale of the cameras, as a percentage of the smallest axis' width. Example: scale = 8 means that the cameras will be around 8% of the smallest axis.
    - `show_cameras::Bool`: determines whether any camera is shown in the figure. 
    - `show_initial_cameras::Bool`: determines if the initial camera poses are shown (only if show_cameras is equal to true).
    - `style::Union{Symbol,Vector{T}} where T<:Real`: determines the linestyle of the trajectories, possible options are: :solid for solid lines, :dash for dashed ones, :dot for dotted one, etc.
    - `theme::String`: set plots theme, by default it's the GLMakie one. Other options are: "presentation" (for theme_ggplot2), "dark" (for theme_black)
    - `trajectories_limits::Vector{T} where T<:Real`: 6-dimensional array defining a box, the trajectories will be plotted only within the box. Order is [x_min, x_max, y_min, y_max, z_min, z_max]. Independent from axes_limits!
"""
function plotCameraTrajectories(traj::Vector{VS.VSTrajectory{VS.spatial}}, O::Union{Nothing,GeometricalObject} = nothing; 
    axes_limits::Vector{T} where T<:Real = [0], colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, color_gradient::Bool = false, 
    desired_pose::Vector{T} where T <: Real = [0], final_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, 
    init_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, object_parameters::NamedTuple = NamedTuple(), scale::Real = 8,
    show_cameras::Bool = true, show_initial_cameras::Bool = true, style::Union{Symbol,Vector{T}} where T<:Real = :solid, theme::String = "",
    trajectories_limits::Vector{T} where T<:Real = [0])

    fig = GLM.Figure();
    ax = GLM.Axis3(fig[1,1], title = "", aspect = :data); # creates figure and axes

    if axes_limits != [0]
        GLM.limits!(ax,axes_limits...); # N.B. settings the limits manually prevents any subsequent, automatic change of limits performed by scatter, lines, etc.
    end

    plotCameraTrajectories!(ax,traj,O,
    color_gradient=color_gradient,colors=colors,desired_pose=desired_pose,final_colors=final_colors,init_colors=init_colors,object_parameters = object_parameters,
    scale=scale,show_cameras=show_cameras,show_initial_cameras=show_initial_cameras,style=style,theme=theme,trajectories_limits=trajectories_limits);

    return fig;
end


"""
    function plotCameraTrajectories!(ax, traj::Union{Vector{VS.VSTrajectory{VS.spatial}},Vector{VS.VSTrajectory{VS.spatialdesiredpose}}}, O::Union{Nothing,GeometricalObject} = nothing; colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, color_gradient::Bool = false, desired_pose::Vector{T} where T <: Real = [0], final_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, init_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, object_parameters::NamedTuple = NamedTuple(), scale::Real = 8, show_cameras::Bool = true, show_initial_cameras::Bool = true, style::Union{Symbol,Vector{T}} where T<:Real = :solid, theme::String = "", trajectories_limits::Vector{T} where T<:Real = [0])    

    Plot the 3D trajectories represented by `traj` on a given axis `ax`.

    # Arguments
    - `ax`: GLMakie.Axis3 where the figure is drawn.
    - `traj::Union{Vector{VS.VSTrajectory{VS.spatial}},Vector{VS.VSTrajectory{VS.spatialdesiredpose}}}`: trajectories to be plotted.
    - `O::GeometricalObject`: tracked 3D object (optional).

    # Keyword arguments
    - `colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol}`: trajectories' colors, can be either one color or a vector of colors. Default colors are the so-called Wong colors (colorblind-friendly)
    - `color_gradient::Bool`: changes the default colors to a gradient of colors
    - `desired_pose::Vector{T} where T<:Real`: add the desired pose to the plot (as a green dotted camera).
    - `final_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol}`: final cameras' colors (same rules as `colors`).
    - `init_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol}`: initial cameras' colors (same rules as `colors`, only if `show_initial_cameras` is active).
    - `object_parameters::NamedTuple` : 3D object plotting parameters.
    - `scale::Real`: scale of the cameras, as a percentage of the smallest axis' width. Example: scale = 8 means that the cameras will be around 8% of the smallest axis.
    - `show_cameras::Bool`: determines whether any camera is shown in the figure. 
    - `show_initial_cameras::Bool`: determines if the initial camera poses are shown (only if show_cameras is equal to true).
    - `style::Union{Symbol,Vector{T}} where T<:Real`: determines the linestyle of the trajectories, possible options are: :solid for solid lines, :dash for dashed ones, :dot for dotted one, etc.
    - `theme::String`: set plots theme, by default it's the GLMakie one. Other options are: "presentation" (for theme_ggplot2), "dark" (for theme_black)
    - `trajectories_limits::Vector{T} where T<:Real`: 6-dimensional array defining a box, the trajectories will be plotted only within the box. Order is [x_min, x_max, y_min, y_max, z_min, z_max]. Independent from axes_limits!
"""
function plotCameraTrajectories!(ax, traj::Union{Vector{VS.VSTrajectory{VS.spatial}},Vector{VS.VSTrajectory{VS.spatialdesiredpose}}}, O::Union{Nothing,GeometricalObject} = nothing; 
    colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, color_gradient::Bool = false, desired_pose::Vector{T} where T <: Real = [0],
    final_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, init_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing,
    object_parameters::NamedTuple = NamedTuple(), scale::Real = 8, show_cameras::Bool = true, show_initial_cameras::Bool = true, style::Union{Symbol,Vector{T}} where T<:Real = :solid,
    theme::String = "", trajectories_limits::Vector{T} where T<:Real = [0])

    inLimits(u) = trajectories_limits[1] <= u[1] <= trajectories_limits[2] && trajectories_limits[3] <= u[2] <= trajectories_limits[4] && trajectories_limits[5] <= u[3] <= trajectories_limits[6];
    
    N = length(traj);
    initShapes = Vector{Vector{Float64}}(undef,N);
    endShapes = Vector{Vector{Float64}}(undef,N);

    # color initialization
    colors = colorsInitialization(colors, N, color_gradient);
    final_colors = isa(final_colors,Nothing) ? colorsInitialization(GLM.RGBA{Float32}(0.0f0,1.0f0,0.0f0,0.3f0),N) : colorsInitialization(final_colors,N);
    init_colors  = isa(init_colors,Nothing)  ? colorsInitialization(GLM.RGBA{Float32}(1.0f0,0.0f0,0.0f0,0.7f0),N) : colorsInitialization(init_colors,N);

    # initialize theme and get points color
    pcolor = themeInitialization(theme);
    !haskey(object_parameters,:color) && (object_parameters = (object_parameters..., color = pcolor));

    for i = 1:N
        initShapes[i] = traj[i].u[1];
        endShapes[i] = traj[i].u[end];
    end

    ax.aspect = :data; # reshapes axes

    if trajectories_limits == [0]
        for i = 1:N
            pts = map(e->GLM.Point3f(e[1:3]),traj[i].u); # converts trajectory to a list of GLMakie points
            GLM.lines!(ax,pts,linewidth=2,color = colors[i], linestyle = style); # plots the points
        end
    else
        for i = 1:N
            k = 1;
            for u in traj[i].u
                if !inLimits(u) || k>length(traj[i].u)
                    break;
                end
                k += 1;
            end
            if k > 1
                pts = map(e->GLM.Point3f(e[1:3]),traj[i].u[1:k-1]); # converts trajectory to a list of GLMakie points
                GLM.lines!(ax, pts, linewidth=2, color = colors[i], linestyle = style); # plots the points
            end
        end
        #pts = map(e->GLM.Point3f(e[1:3]),initShapes); # converts starting points to a list of GLMakie points
        #GLM.scatter!(ax,pts,marker=:xcross,markersize=10,color=:black); # plots the points
    end

    # plot the 3D object, if provided
    !isa(O, Nothing) && (plotObject!(ax,O,object_parameters));

    # camera shape definition
    v,f,sc = getCameraShape();

    R(x) = RC.quaternionToRotationMatrix(x[4:end]);
    p(x) = x[1:3];

    GLM.reset_limits!(ax); # explicitly asks Makie to compute the axis limits based on the plot content

    cameraScale = scale*maximum(ax.finallimits[].widths)/100; # by default, camera is ~8% w.r.t. the largest axis width

    if desired_pose != [0]
        GLM.poly!(ax, transpose((R(desired_pose)*transpose(v).*cameraScale).+p(desired_pose)), f, 
        strokewidth = 2.5, linestyle = :dash, color = GLM.RGBA{Float32}(51/255,100/255,82/255,0.3f0), strokecolor = sc);
    end

    if show_cameras
        for i = 1:N
            show_initial_cameras && GLM.poly!(ax,transpose((R(initShapes[i])*transpose(v).*(cameraScale*0.75)).+p(initShapes[i])), f, strokewidth = 2.5, color = init_colors[i], strokecolor = sc);
            GLM.poly!(ax,transpose((R(endShapes[i])*transpose(v).*cameraScale).+p(endShapes[i])), f, strokewidth = 2.5, color = final_colors[i], strokecolor = sc);
        end
    end
end

"""
    function plotImageTrajectories(traj::Union{Vector{VS.VSTrajectory{VS.spatial}},Vector{VS.VSTrajectory{VS.spatialdesiredpose}}}, P::AbstractMatrix; colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, color_gradient::Bool = true, desired_pose::Vector{T} where T <: Real = [0], final_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, frame_thickness::Real = 0.5, init_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, initial_point_size::Union{Integer,Nothing} = nothing, lines_order::Union{Nothing,Vector{T} where T<:Integer} = nothing, lines_thickness::Real = 2, point_size::Integer = 20, screen_limits::Vector{T} where T<:Real = [0], show_frame::Bool = false, show_initial_frame::Bool = false, show_initial_pose::Bool = false, style::Union{Symbol,Vector{T}} where T<:Real = :solid, theme::String = "", tracks_thickness::Real = 1)

    Plot the trajectories represented by `traj` in the features space (i.e., as seen on the camera screen).

    # Arguments
    - `traj::Union{Vector{VS.VSTrajectory{VS.spatial}},Vector{VS.VSTrajectory{VS.spatialdesiredpose}}}`: trajectories to be plotted.
    - `P::AbstractMatrix`: matrix of tracking points.

    # Keyword arguments
    - `colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol}`: trajectories' colors, can be either one color or a vector of colors. Default colors are the so-called Wong colors (colorblind-friendly)
    - `color_gradient::Bool`: changes the default colors to a gradient of colors 
    - `desired_pose::Vector{T} where T <: Real`: if given, it shows the desired pose in green
    - `final_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol}`: final images' colors (same rules as `colors`).
    - `frame_thickness::Real`: [SOON TO BE DEPRECATED] wireframe thickness (only if `show_frame` is active, see below). 
    - `init_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol}`: initial images' colors (same rules as `colors`, only if `show_initial_pose` is active).
    - `initial_point_size::Union{Integer,Nothing}`: initial images' tracking points size. By default, equal to `point_size` (see below).
    - `lines_order::Union{Nothing,Vector{T} where T<:Integer}`: changes the shape of the tracking points, from points to a 3D figure: it is a vector that defines in which order to write the lines. Example: the P matrix defines a square, lines_order might be [1,2,3,4,1], meaning that we draw lines connecting the first point to the second, the second to the third, etc., connecting back to the first one in the end to create a square.
    - `lines_thickness::Real`: thickness of the lines defined with lines_order
    - `point_size::Real`: tracking points size
    - `screen_limits::Vector{T} where T<:Real`: 4-dimensional array to set the axes limits. Order is [x_min, x_max, y_min, y_max].
    - `show_frame::Bool`: [SOON TO BE DEPRECATED] adds a wireframe around the tracking points, for giving a 3D "impression" and better aesthetics. 
    - `show_initial_frame::Bool`: [SOON TO BE DEPRECATED] adds a wireframe for the initial images as well. 
    - `show_initial_pose::Bool`: determines if the initial camera images are shown. 
    - `style::Union{Symbol,Vector{T}} where T<:Real`: determines the linestyle of the trajectories, possible options are: :solid for solid lines, :dash for dashed ones, :dot for dotted one, etc.
    - `theme::String`: set plots theme, by default it's the GLMakie one. Other options are: "presentation" (for theme_ggplot2), "dark" (for theme_black)
    - `tracks_thickness::Real`: line thickness of the trajectories in the image.
"""
function plotImageTrajectories(traj::Union{Vector{VS.VSTrajectory{VS.spatial}},Vector{VS.VSTrajectory{VS.spatialdesiredpose}}}, P::AbstractMatrix; 
    colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, color_gradient::Bool = true, 
    desired_pose::Vector{T} where T <: Real = [0], final_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, frame_thickness::Real = 0.5, 
    init_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, initial_point_size::Union{Integer,Nothing} = nothing,
    lines_order::Union{Nothing,Vector{T} where T<:Integer} = nothing, lines_thickness::Real = 2, point_size::Integer = 20,
    screen_limits::Vector{T} where T<:Real = [0], show_frame::Bool = false, show_initial_frame::Bool = false, show_initial_pose::Bool = false, 
    style::Union{Symbol,Vector{T}} where T<:Real = :solid, theme::String = "", tracks_thickness::Real = 1)

    fig = GLM.Figure();
    ax = GLM.Axis(fig[1,1], title = "", aspect = GLM.DataAspect()); # creates figure and axes

    if screen_limits != [0]
        GLM.limits!(ax,screen_limits...); # N.B. settings the limits manually prevents any subsequent, automatic change of limits performed by scatter, lines, etc.
    end

    plotImageTrajectories!(ax,traj,P,colors=colors,color_gradient=color_gradient,desired_pose=desired_pose,final_colors=final_colors,frame_thickness=frame_thickness,
        init_colors=init_colors,initial_point_size=initial_point_size,lines_order=lines_order,lines_thickness=lines_thickness,point_size=point_size,show_frame=show_frame,
        show_initial_frame=show_initial_frame,show_initial_pose=show_initial_pose,style=style,theme=theme,tracks_thickness=tracks_thickness);
    
    return fig;
end

"""
    function plotImageTrajectories!(ax, traj::Union{Vector{VS.VSTrajectory{VS.spatial}},Vector{VS.VSTrajectory{VS.spatialdesiredpose}}}, P::AbstractMatrix; colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, color_gradient::Bool = true, desired_pose::Vector{T} where T <: Real = [0], final_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, frame_thickness::Real = 0.5, init_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, initial_point_size::Union{Integer,Nothing} = nothing, lines_order::Union{Nothing,Vector{T} where T<:Integer} = nothing, lines_thickness::Real = 2, point_size::Integer = 20, show_frame::Bool = true, show_initial_frame::Bool = false, show_initial_pose::Bool = false, style::Union{Symbol,Vector{T}} where T<:Real = :solid, theme::String = "", tracks_thickness::Real = 1)

    Plot the trajectories represented by `traj` in the features space (i.e., as seen on the camera screen), on a given axis `ax`.

    # Arguments
    - `ax`: GLMakie.Axis where the figure is drawn.
    - `traj::Union{Vector{VS.VSTrajectory{VS.spatial}},Vector{VS.VSTrajectory{VS.spatialdesiredpose}}}`: trajectories to be plotted.
    - `P::AbstractMatrix`: matrix of tracking points.

    # Keyword arguments
    - `colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol}`: trajectories' colors, can be either one color or a vector of colors. Default colors are the so-called Wong colors (colorblind-friendly)
    - `color_gradient::Bool`: changes the default colors to a gradient of colors 
    - `desired_pose::Vector{T} where T <: Real`: if given, it shows the desired pose in green
    - `final_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol}`: final images' colors (same rules as `colors`).
    - `frame_thickness::Real`: [SOON TO BE DEPRECATED] wireframe thickness (only if `show_frame` is active, see below). 
    - `init_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol}`: initial images' colors (same rules as `colors`, only if `show_initial_pose` is active).
    - `initial_point_size::Union{Integer,Nothing}`: initial images' tracking points size. By default, equal to `point_size` (see below).
    - `lines_order::Union{Nothing,Vector{T} where T<:Integer}`: changes the shape of the tracking points, from points to a 3D figure: it is a vector that defines in which order to write the lines. Example: the P matrix defines a square, lines_order might be [1,2,3,4,1], meaning that we draw lines connecting the first point to the second, the second to the third, etc., connecting back to the first one in the end to create a square.
    - `lines_thickness::Real`: thickness of the lines defined with lines_order
    - `point_size::Real`: tracking points size
    - `show_frame::Bool`: [SOON TO BE DEPRECATED] adds a wireframe around the tracking points, for giving a 3D "impression" and better aesthetics. 
    - `show_initial_frame::Bool`: [SOON TO BE DEPRECATED] adds a wireframe for the initial images as well. 
    - `show_initial_pose::Bool`: determines if the initial camera images are shown. 
    - `style::Union{Symbol,Vector{T}} where T<:Real`: determines the linestyle of the trajectories, possible options are: :solid for solid lines, :dash for dashed ones, :dot for dotted one, etc.
    - `theme::String`: set plots theme, by default it's the GLMakie one. Other options are: "presentation" (for theme_ggplot2), "dark" (for theme_black)
    - `tracks_thickness::Real`: line thickness of the trajectories in the image.
"""
function plotImageTrajectories!(ax, traj::Union{Vector{VS.VSTrajectory{VS.spatial}},Vector{VS.VSTrajectory{VS.spatialdesiredpose}}}, P::AbstractMatrix; 
    colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, color_gradient::Bool = true, desired_pose::Vector{T} where T <: Real = [0], 
    final_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, frame_thickness::Real = 0.5, 
    init_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, initial_point_size::Union{Integer,Nothing} = nothing, 
    lines_order::Union{Nothing,Vector{T} where T<:Integer} = nothing, lines_thickness::Real = 2, point_size::Integer = 20, show_frame::Bool = true, 
    show_initial_frame::Bool = false, show_initial_pose::Bool = false, style::Union{Symbol,Vector{T}} where T<:Real = :solid, 
    theme::String = "", tracks_thickness::Real = 1)

    #=======

    General setup (definition of colors and theme, figures creation, etc.)

    =======#
    
    ax.yreversed = true; # enforces that the camera image matches the "true" camera PoV

    if !isa(lines_order,Nothing) && (show_frame || show_initial_frame)
        @warn "Having both the wire-frame and the interconnecting lines might lead to a cluttered screen; the wire-frame is therefore disabled.";
        show_frame = false;
        show_initial_frame = false;
    end

    s(x,P) = (Pc = transpose(RC.quaternionToRotationMatrix(x[4:end]))*(P.-x[1:3]); return [Pc[1,:]./Pc[3,:], Pc[2,:]./Pc[3,:]]);
    s_p(x,P) = (Pc = transpose(RC.quaternionToRotationMatrix(x[4:end]))*(P.-x[1:3]); return [GLM.Point2f(x,y) for (x,y) in zip(Pc[1,:]./Pc[3,:], Pc[2,:]./Pc[3,:])]);

    show_frame && (P_frame = getPointsFrame(P));
    isa(initial_point_size,Nothing) && (initial_point_size = point_size);

    n = length(traj);

    # colors initialization
    colors = colorsInitialization(colors, n, color_gradient);
    final_colors = isa(final_colors,Nothing) ? copy(colors) : colorsInitialization(final_colors,n);
    init_colors  = isa(init_colors,Nothing)  ? copy(colors) : colorsInitialization(init_colors,n);

    # initialize theme and get points color
    themeInitialization(theme);

    #=======

    Plot static stuff

    =======#

    
    for i in eachindex(traj)
        if isa(lines_order,Nothing)
            GLM.scatter!(ax, s_p(traj[i].u[end],P), color = final_colors[i], markersize = point_size);
        else
            GLM.lines!(ax, s_p(traj[i].u[end],P)[lines_order], color = final_colors[i], linewidth = lines_thickness);
        end
        
        if show_frame
            s_tf = s(traj[i].u[end], P_frame);
            GLM.lines!(ax,s_tf[1], s_tf[2], linewidth = frame_thickness, color = final_colors[i]);
        end
        
        if show_initial_pose
            if isa(lines_order,Nothing)
                GLM.scatter!(ax, s_p(traj[i].u[1],P), color = init_colors[i], markersize = initial_point_size, marker = :cross);
            else
                GLM.lines!(ax, s_p(traj[i].u[1],P)[lines_order], color = init_colors[i], linewidth = lines_thickness);
            end

            if show_initial_frame
                s_tf = s(traj[i].u[1], P_frame);
                GLM.lines!(ax,s_tf[1], s_tf[2], linewidth = frame_thickness, color = init_colors[i]);
            end
        end
    end

    if desired_pose != [0]
        desired_color = GLM.RGBA{Float32}(51/255,100/255,82/255,0.8f0);
        s_tmp = s(desired_pose,P);
        GLM.scatter!(ax, s_tmp[1], s_tmp[2], marker = :cross,
        markersize = point_size+2, color = desired_color);

        if show_frame_in_screen
            s_tf = s(desired_pose,P_frame);
            GLM.lines!(ax,s_tf[1], s_tf[2], linewidth = frame_thickness, color = desired_color);
        end
    end

    #=========

    Plot trajectories

    =========#

    tracks = Vector{Vector{Vector{GLM.Point2f}}}(undef,n); # trajectories -> tracking points -> tracks

    for i in eachindex(traj)
        tracks[i] = [Vector{GLM.Point2f}(undef,length(traj[i].u)) for _ in axes(P,2)];

        for j in eachindex(traj[i].u)
            s_tmp = s_p(traj[i].u[j],P);
            
            for k in eachindex(s_tmp)
                tracks[i][k][j] = copy(s_tmp[k]);
            end
        end

        for track in tracks[i]
            GLM.lines!(ax, track, linewidth = tracks_thickness, color = colors[i], linestyle = style);
        end
    end
end

#
function showTrajectoryVideo(traj::Union{Vector{VS.VSTrajectory{VS.spatial}},Vector{VS.VSTrajectory{VS.spatialdesiredpose}}}, P::AbstractMatrix = [0 0]; 
    axes_limits::Vector{T} where T<:Real = [0], azimuth = nothing, colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, color_gradient::Bool = false, 
    desired_pose::Vector{T} where T <: Real = [0], elevation = nothing, filename::Union{String,Nothing} = nothing, final_time::Real = 0, initial_time::Real = 0., lines_order::Union{Nothing,Vector{T} where T<:Integer} = nothing, lines_thickness::Real = 2,
    live_video::Bool = false, point_size::Integer = 20, scale::Real=4, screen_limits::Vector{T} where T<:Real = [0], show_frame_in_3D::Bool = false, show_frame_in_screen::Bool = false, 
    show_tracks_in_3D::Bool = true, show_tracks_in_screen::Bool = true, show_screen::Bool = false, theme::String = "", title::String = "", video_duration::Real = 30, 
    video_resolution::Tuple{Int64, Int64} = (1440,1080))

    #=======

    General setup (definition of colors and theme, figures creation, etc.)

    =======#

    # unconsistency checks
    (show_screen && P == [0 0]) && (@error "P matrix is needed to show the points on the screen.");

    show_lines = !isa(lines_order,Nothing);
    points_order = show_lines ? map(i->findfirst(e->e==i,lines_order),1:size(P,2)) : [1:size(P,2)...]; # "re-order" points order from show_lines; used to properly define points' tracks in the screen (below)

    if show_lines && (show_frame_in_screen || show_frame_in_3D)
        @warn "Having both the wire-frame and the interconnecting lines might lead to a cluttered screen; the wire-frame is therefore disabled.";
        show_frame_in_screen = false;
        show_frame_in_3D = false;
    end

    # features evaluation, both in form of a matrix or a list of 2D points
    s(x,P) = (Pc = transpose(RC.quaternionToRotationMatrix(x[4:end]))*(P.-x[1:3]); return [Pc[1,:]./Pc[3,:], Pc[2,:]./Pc[3,:]]);
    s_p(x,P) = (Pc = transpose(RC.quaternionToRotationMatrix(x[4:end]))*(P.-x[1:3]); return [GLM.Point2f(x,y) for (x,y) in zip(Pc[1,:]./Pc[3,:], Pc[2,:]./Pc[3,:])]);

    (show_frame_in_3D || show_frame_in_screen) && (P_frame = getPointsFrame(P));

    time = GLM.Observable(Float64(initial_time));
    n = length(traj);
    
    # colors initialization
    colors = colorsInitialization(colors, n, color_gradient);

    # initialize theme and get points color
    pcolor = themeInitialization(theme);

    # figure and axes creation
    fig = GLM.Figure(size = video_resolution);

    if show_screen
        ax = GLM.Axis3(fig[1,1], title = "3D view", aspect = :data);
        ax_screen = GLM.Axis(fig[1,0], title = "Camera view", aspect = GLM.DataAspect(), yreversed = true);
        GLM.Label(fig[0,0:1], ( title == "" ? GLM.@lift("t = $(round($time, digits = 1)) s") : title), font=:bold, fontsize=20);
    else
        ax = GLM.Axis3(fig[1,1], title = ( title == "" ? GLM.@lift("t = $(round($time, digits = 1)) s") : title), aspect = :data);
    end

    !isa(azimuth,Nothing) && (ax.azimuth = azimuth);
    !isa(elevation,Nothing) && (ax.elevation = elevation);

    #=======

    Automatic resizing of axes limits, based on trajectories largest values

    =======#
 
    # "reduce" finds the min/max "row-wise", then we take just xyz and vectorize to have the correct size
    # "copy" serves the purpose of casting back to Vector{Float/Int} instead of reshape(ajoint(...))
    #limits = copy(vec([(reduce((x,y) -> min.(x,y), [q for tr in traj for q in tr.u]).-1)[1:3] (reduce((x,y) -> max.(x,y), [q for tr in traj for q in tr.u]).+1)[1:3]]'));

    # define interpolated trajectories
    intTrs = map(i->TrajectoryInterpolator(traj[i].u,traj[i].t),1:n);
    
    tmax = (final_time == 0) ? maximum([traj[i].t[end] for i in eachindex(traj)]) : final_time;
    framerate = 30;
    timestamps = range(initial_time, tmax, length = framerate*video_duration);

    al = (axes_limits == [0]);
    sl = (show_screen && screen_limits == [0]);

    # the following if manages the automatic resizing of the axes
    # it is badly structured, but it manages to read all trajectories only once
    if al || sl
        al && (limits = [Inf,-Inf,Inf,-Inf,Inf,-Inf]); # inline ifs
        sl && (c_limits = [Inf,-Inf,Inf,-Inf]);

        for tr in intTrs
            for t in timestamps
                tmp=tr(t);
                s_tmp = s(tmp,P);

                for i in 1:3
                    al && ( limits[2*i-1:2*i] .= (min(limits[2*i-1],tmp[i]),max(limits[2*i],tmp[i])) );
                    (sl && (i != 3)) && ( c_limits[2*i-1:2*i] .= (minimum([s_tmp[i]...,c_limits[2*i-1]]),maximum([s_tmp[i]...,c_limits[2*i]])) );
                end
            end

            tr(tr.t[end]+1); # reset trajectory's counter; TrajectoryInterpolator is made for sequential access, see Plotter.jl for implementation
        end

        if al
            cameraScale = scale*maximum([limits[i+1]-limits[i] for i in [1,3,5]])/100;
            limits += [-2*cameraScale,2*cameraScale,-2*cameraScale,2*cameraScale,-2*cameraScale,2*cameraScale]; # add a bit of padding around the plot
        else
            limits = copy(axes_limits);
            cameraScale = scale*maximum([limits[i+1]-limits[i] for i in [1,3,5]])/(100+4*scale);
        end

        if sl
            c_widths = 0.01*[c_limits[i+1]-c_limits[i] for i in [1,3]];
            c_limits += [-c_widths[1],c_widths[1],-c_widths[2],c_widths[2]];
        else
            c_limits = copy(screen_limits);
        end

    else
        limits = copy(axes_limits);
        c_limits = copy(screen_limits);
        cameraScale = scale*maximum([limits[i+1]-limits[i] for i in [1,3,5]])/(100+4*scale);
    end
        
    GLM.limits!(ax,limits...);
    show_screen && GLM.limits!(ax_screen,c_limits...); # inline if!

    #println("DEBUG : ", ax.finallimits[]);


    #=======
    
    Plot static stuff
    
    =======#

    if P != [0 0]
        if !show_lines
            GLM.scatter!(ax,P[1,:], P[2,:], P[3,:], markersize=point_size,color=pcolor); # plots tracking points
        else
            GLM.lines!(ax, P[1,:][lines_order], P[2,:][lines_order], P[3,:][lines_order], color = pcolor, linewidth = lines_thickness);
        end
        
        if show_frame_in_3D
            c = pcolor == :black ? 0.0f0 : 1.0f0;
            pcolor_faded = GLM.RGBA{Float32}(c,c,c,0.4f0);
            GLM.lines!(ax,map(i->GLM.Point3f(P_frame[:,i]),axes(P_frame,2)), linewidth=0.5, color=pcolor_faded);
        end
    end

    # camera shape definition
    v,f,sc = getCameraShape();

    R(x) = RC.quaternionToRotationMatrix(x[4:end]);
    p(x) = x[1:3];    

    if desired_pose != [0]
        GLM.poly!(ax, transpose((R(desired_pose)*transpose(v).*cameraScale).+p(desired_pose)), f, 
        strokewidth = 2.5, linestyle = :dash, color = GLM.RGBA{Float32}(51/255,100/255,82/255,0.3f0), strokecolor = sc);

        if show_screen
            s_tmp = s(desired_pose,P);
            if !show_lines
                GLM.scatter!(ax_screen, s_tmp[1], s_tmp[2], marker = :cross,
                markersize = point_size+2, color = GLM.RGBA{Float32}(51/255,100/255,82/255,0.8f0));
            else
                GLM.lines!(ax_screen, s_tmp[1][lines_order], s_tmp[2][lines_order], color =  GLM.RGBA{Float32}(51/255,100/255,82/255,0.8f0), linewidth = lines_thickness);
            end

            if show_frame_in_screen
                s_tf = s(desired_pose,P_frame);
                GLM.lines!(ax_screen,s_tf[1], s_tf[2], linewidth = 0.5, color = GLM.RGBA{Float32}(51/255,100/255,82/255,0.8f0))
            end
        end
    end

    #=======

    Initialize video stuff

    =======#

    # filename
    filename = isa(filename,Nothing) ? "videos/spatial_"*Dates.format(Dates.now(), "yyyymmdd-HH:MM")*".mp4" : "videos/"*filename*".mp4";
    mkpath(dirname(filename));
    
    # cameras and tracks "objects", comments below are "element-wise"
    cameras = Vector{GLM.Observable}(undef,n); # matrix, depending on time; each column is a vertex of the camera shape
    tracks = Vector{GLM.Observable}(undef,n); # list of 3D points, a point is added after each time-step; each point is the camera's position at a given time instant
    features = Vector{GLM.Observable}(undef,n); # list of N 2D points, depending on time; each point is one of the tracking points' features
    screen_tracks = Vector{Vector{GLM.Observable}}(undef,n); # vector of obs. of length N, each element is a list of 2D points that increases at each time-step

    show_frame_in_screen && (frame_features = Vector{GLM.Observable}(undef,n)); # 

    for i in eachindex(intTrs)
        cameras[i] = GLM.@lift transpose((R(intTrs[i]($time))*transpose(v).*cameraScale).+p(intTrs[i]($time)));
        tracks[i] = GLM.Observable([GLM.Point3f(intTrs[i](time[]))]);

        GLM.poly!(ax, cameras[i], f, strokewidth = 2.5, color = colors[i], strokecolor = sc);
        GLM.lines!(ax, tracks[i], linewidth = 2, color = colors[i]);

        if show_screen
            if !show_lines
                features[i] = GLM.@lift s_p(intTrs[i]($time),P);
                GLM.scatter!(ax_screen, features[i], color = colors[i], markersize = point_size);
            else
                features[i] = GLM.@lift s_p(intTrs[i]($time),P)[lines_order];
                GLM.lines!(ax_screen, features[i], color = colors[i], markersize = point_size);
            end
            screen_tracks[i] = Vector{GLM.Observable}(undef,size(P,2));

            for j in 1:size(P,2)
                screen_tracks[i][j] = GLM.Observable([features[i][][points_order[j]]]);
                GLM.lines!(ax_screen, screen_tracks[i][j], linewidth = 1, color = colors[i], linestyle = :dash);
            end
            
            if show_frame_in_screen
                frame_features[i] = GLM.@lift s_p(intTrs[i]($time),P_frame);
                GLM.lines!(ax_screen, frame_features[i], color = colors[i], linewidth = 0.5);
            end
        end
    end

    #=======

    Video plotting

    =======#

    if live_video
        display(fig);

        for t in timestamps
            time[] = t;
            if show_tracks_in_3D
                for i = 1:n
                    push!(tracks[i][], GLM.Point3f(intTrs[i](t)));
                    notify(tracks[i]);
                end
            end
            if show_screen && show_tracks_in_screen
                for i = 1:n
                    for j = 1:size(P,2)
                        push!(screen_tracks[i][j][], features[i][][points_order[j]]);
                        notify(screen_tracks[i][j]);
                    end
                end
            end
            sleep(1/(1.5*framerate));
        end
    else
        # video creation
        GLM.record(fig, filename, timestamps;
                framerate = framerate) do t
                    
            time[] = t;
            if show_tracks_in_3D
                for i = 1:n
                    push!(tracks[i][], GLM.Point3f(intTrs[i](t)));
                    notify(tracks[i]);
                end
            end
            if show_screen && show_tracks_in_screen
                for i = 1:n
                    for j = 1:size(P,2)
                        push!(screen_tracks[i][j][], features[i][][points_order[j]]);
                        notify(screen_tracks[i][j]);
                    end
                end
            end
        end
    end
    
end

function plotCameras(oxcs, P::AbstractMatrix; scale=4, points_size = 15, cls=GLM.RGBA{Float32}(0.0f0,1.0f0,0.0f0,1.0f0),theme="",barycenter = "back")
    @warn "The passage of an explicit matrix of points P has been deprecated. A GeometricalObject should be provided instead."

    O = Points(P);
    object_parameters = (points_size = points_size,);

    return plotCameras(oxcs,O=O,cls=cls,object_parameters=object_parameters,scale=scale,theme=theme,barycenter=barycenter)
end

function plotCameras(oxcs; O::Union{GeometricalObject,Nothing} = nothing, cls=GLM.RGBA{Float32}(0.0f0,1.0f0,0.0f0,1.0f0), object_parameters::NamedTuple = NamedTuple(), scale=4, theme="", barycenter = "back")
    fig = GLM.Figure();
    ax = GLM.Axis3(fig[1,1], title = "", aspect = :data); # creates figure and axes

    plotCameras!(ax,oxcs,O=O,scale=scale,object_parameters=object_parameters,cls=cls,theme=theme,barycenter=barycenter);

    return fig;
end

function plotCameras!(ax, oxcs, P::AbstractMatrix; scale=4, points_size = 15, cls=GLM.RGBA{Float32}(0.0f0,1.0f0,0.0f0,1.0f0),theme="",resize_axes = true,barycenter = "back")
    @warn "The passage of an explicit matrix of points P has been deprecated. A GeometricalObject should be provided instead."

    O = Points(P);
    object_parameters = (points_size = points_size,);

    plotCameras!(ax,oxcs,O=O,cls=cls,object_parameters=object_parameters,scale=scale,theme=theme,resize_axes=resize_axes,barycenter=barycenter)
end

function plotCameras!(ax, oxcs; O::Union{GeometricalObject,Nothing} = nothing, barycenter = "back", cls = GLM.RGBA{Float32}(0.0f0,1.0f0,0.0f0,1.0f0), object_parameters::NamedTuple = NamedTuple(), scale = 4, resize_axes = true, theme = "")
    if isa(cls,Vector) && length(cls) != length(oxcs)
        @warn "Dimension mismatch; only the first color will be used.";
        cls = cls[1];
    end

    # initialize theme and get points color
    pcolor = themeInitialization(theme);
    !haskey(object_parameters,:color) && (object_parameters = (object_parameters..., color = pcolor));

    function largestDistance(v)
        return maximum(v) - minimum(v);
    end
    
    coordinates = isa(O,Points) ? [[x[j] for x in [oxcs...,[O[:,i] for i in axes(O,2)]...]] for j in 1:3] : [[x[j] for x in oxcs] for j in 1:3];
    s = scale*(1+maximum(map(largestDistance, coordinates)))/100; # by default, camera is ~4% w.r.t. the largest axis width
    
    v,f,sc = getCameraShape(barycenter);

    R(x) = RC.quaternionToRotationMatrix(x[4:end]);
    p(x) = x[1:3];

    ax.aspect = :data; # reshapes axes

    if !isa(cls,Vector)
        for x in oxcs
            GLM.poly!(ax,transpose((R(x)*transpose(v).*s).+p(x)), f, strokewidth = 2.5, color = cls, strokecolor = sc);
        end
    else
        for i in eachindex(oxcs)
            GLM.poly!(ax,transpose((R(oxcs[i])*transpose(v).*s).+p(oxcs[i])), f, strokewidth = 2.5, color = cls[i], strokecolor = sc);
        end
    end

    if resize_axes
        # what follows is a hack to give the plot a "cubic" shape
        GLM.reset_limits!(ax); # updates axes limits to fit the content of ax
        L = argmax(ax.finallimits[].widths);
        M = ax.finallimits[].widths[L]/2;

        limits = Vector{Float64}(undef,6);

        for i = 1:3
            m = ax.finallimits[].origin[i] + ax.finallimits[].widths[i]/2;
            limits[2*i-1:2*i] = [m-M, m+M];
        end

        GLM.limits!(ax,limits...);
    end

    # plot the 3D object, if provided
    !isa(O, Nothing) && (plotObject!(ax,O,object_parameters));
end

function plotScreen(oxcs::Vector{T} where T<:AbstractVector, P::AbstractMatrix; 
    colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, color_gradient::Bool = false,
    lines_order::Union{Nothing,Vector{T} where T<:Integer} = nothing, lines_thickness::Real = 2,
    point_size::Real = 20, theme="")
    fig = GLM.Figure();
    ax = GLM.Axis(fig[1,1], title = "", aspect = GLM.DataAspect()); # creates figure and axes

    plotScreen!(ax,oxcs,P,colors=colors,color_gradient=color_gradient,lines_order=lines_order,lines_thickness=lines_thickness,point_size=point_size,theme=theme);

    return fig;
end

function plotScreen!(ax, oxcs::Vector{T} where T<:AbstractVector, P::AbstractMatrix; 
    colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, color_gradient::Bool = false,
    lines_order::Union{Nothing,Vector{T} where T<:Integer} = nothing, lines_thickness::Real = 2,
    point_size::Real = 20, theme="")

    ax.yreversed = true; # enforces that the camera image matches the "true" camera PoV

    n = length(oxcs);
    show_lines = !isa(lines_order, Nothing);

    # colors initialization
    colors = colorsInitialization(colors, n, color_gradient);

    # initialize theme and get points color
    themeInitialization(theme);

    s_p(x,P) = (Pc = transpose(RC.quaternionToRotationMatrix(x[4:end]))*(P.-x[1:3]); return [GLM.Point2f(x,y) for (x,y) in zip(Pc[1,:]./Pc[3,:], Pc[2,:]./Pc[3,:])]);

    for (i,x) in enumerate(oxcs)
        if show_lines
            GLM.lines!(ax, s_p(x,P)[lines_order], color = colors[i], linewidth = lines_thickness);
        else
            GLM.scatter!(ax, s_p(x,P), color = colors[i], markersize = point_size);
        end
    end
end

function getCameraShape(barycenter = "back")
    vr = 0.5;
    #v = [-vr vr 1.;vr vr 1.;vr -vr 1.;-vr -vr 1.;0. 0. -1.]; # "slimmer" camera
    v_back = [-vr vr 0.;vr vr 0.;vr -vr 0.;-vr -vr 0.;0. 0. -2.]; # "pushed back", "slimmer" camera
    v_front = [-vr vr 2.;vr vr 2.;vr -vr 2.;-vr -vr 2.;0. 0. 0.]; # "pushed forward", "slimmer" camera
    v = (barycenter == "back") ? v_back : v_front;

    f = [1 2 3;3 4 1;1 5 2;2 5 3;3 5 4;4 5 1]; # order of vertices to plot the camera

    # black magic to not plot a black line across the camera screen
    # found by trial-and-error, not sure why or how it works
    sc = [repeat([:transparent],16)...,repeat([:black],20)...];

    return v,f,sc;
end

function getPointsFrame(P::AbstractMatrix)
	l,u = Des.getEnclosingRectangle(P);

    # add a bit of padding
	w_m = 0.08*maximum(u-l);
    l .-= w_m;
    u .+= w_m;

    w = u - l; # vector of widths

    sel = [[0,0,0],[1,0,0],[1,1,0],[0,1,0],
     	   [0,1,1],[0,0,1],[1,0,1],[1,1,1]]; # vertex "relative definition" w.r.t. the vertex l

    ord = [1,2,3,4,1,6,7,8,5,6,7,2,3,8,5,4]; # vertices' plotting order, to plot all the sides

    Pf = zeros(size(P,1),length(ord)); # matrix containing all the vertices in order

    for i in eachindex(ord)
        Pf[:,i] = l + w.*sel[ord[i]];
    end

    return Pf;
end

function colorsInitialization(colors, n, color_gradient = false)
    if isa(colors,Nothing) # default colors
        colors = color_gradient ? getColorShades(n) : repeat(GLM.Makie.wong_colors(),n÷7+1)[1:n];
    elseif isa(colors,Union{GLM.ColorAlpha, GLM.Color, Symbol})
        colors = [colors for _ in 1:n];
    elseif isa(colors,Vector) && length(colors) != n
        @warn "Dimension mismatch (trajectories: $n, colors: $(length(colors))); only the first color will be used.";
        colors = [colors[1] for _ in 1:n];
    end

    return colors;
end

function styleInitialization(style, n)
    if isa(style, Symbol)
        style = repeat([style],n);
    elseif isa(style, Vector{Symbol})
        if length(style) != n
            @warn "Dimension mismatch (trajectories: $n, styles: $(length(style))); only the first color will be used.";
            style = repeat(style[1:1],n);
        end
    else
        @error "Other style-types are not supported yet, you lazy bum."
    end

    return style;
end

function themeInitialization(theme)
    if theme == "dark"
        GLM.set_theme!(GLM.theme_black())
        pcolor = :white;
    elseif theme == "presentation"
        GLM.set_theme!(GLM.theme_ggplot2()) #ggplot2
        pcolor = :black;
    else
        GLM.set_theme!() # default glmakie theme
        pcolor = :black;
    end

    return pcolor;
end

"""
    function showLayedOutVideo(traj::Vector{VS.VSTrajectory{VS.spatial}}, axes_layout, P::AbstractMatrix = [0 0]; column_widths = nothing, desired_pose::Vector{T} where T <: Real = [0], filename::Union{String,Nothing} = nothing, final_time::Real = 0, initial_time::Real = 0., framerate::Integer = 30, interpolation_factor::Integer = 1, lines_order::Union{Nothing,Vector{T} where T<:Integer} = nothing, lines_thickness::Real = 2, live_video::Bool = false, logarithmic_time::Bool = false, row_widths = nothing, show_tracks_in_3D::Bool = true, show_tracks_in_screen::Bool = true, theme::String = "", title::Union{String,GLM.Makie.LaTeXStrings.LaTeXString} = "", video_duration::Real = 30, video_resolution::Tuple{Int64, Int64} = (1440,1080))
    
    Creates a video with possibly several "views", as defined by `axes_layout`, both in state space and features space.

    # Arguments
    - `traj::Vector{VS.VSTrajectory{VS.spatial}}`: trajectories to be plotted.
    - `axes_layout`: list of named tuples, describing each of the video's views. Each element must containt the following keys:
      + `type` : determines if the view is in state space ("camera") or in features space ("image").
      + `list` : determines the subset of trajectories shown in the view. Can be a vector of Integers or a Range (e.g., 1:3).
      + `axes_position` : determines the view's position in the figure (e.g., (1:2, 2)).
      + `settings` : a dictionary of settings specific for the view (see `initializeImageVideo!` and `initializeCameraVideo!`).
      Example:
        axes_layout = [
        (type = "camera", list = 1:4, axes_position = (1:2,1),
                settings = Dict(:color_gradient => true, :scale => 10)),
        (type = "image", list = [1], axes_position = (1,2),
                settings = Dict(:colors => :red, title => "Test title"))];
        
    # Keyword arguments
    - `O::Union{GeometricalObject,Nothing}` : tracked 3D object.
    - `column_widths` : vector of Floats, sets the width of each column (in percentage).
    - `desired_pose::Vector{T} where T <: Real` : camera's desired pose, statically plotted if given.
    - `filename::Union{String,Nothing}` : filename of video (if live_video = false).
    - `final_time::Real` : final instant in simulation time.
    - `initial_time::Real` : initial instant in simulation time.
    - `framerate::Integer` : video framerate (if live_video = false).
    - `interpolation_factor::Integer` : number of points plotted between two instants of time.
    - `live_video::Bool` : discriminates between showing the video on screen or saving to file.
    - `logarithmic_time::Bool` : time goes faster as it increases, rendering exponential convergence (almost) linear.
    - `object_parameters::NamedTuple` : 3D object's parameters.
    - `row_widths` : vector of Floats, sets the width of each row (in percentage).
    - `show_tracks_in_3D::Bool` : flag to plot a persistent "tail" representing the cameras trajectories (in 3D).
    - `show_tracks_in_screen::Bool` : flag to plot a persistent "tail" representing the points trajectories (in features space).
    - `theme::String` : set plots theme, by default it's the GLMakie one. Other options are: "presentation" (for theme_ggplot2), "dark" (for theme_black)
    - `title::Union{String,GLM.Makie.LaTeXStrings.LaTeXString}` : figure title.
    - `video_duration::Real` : video duration in seconds.
    - `video_resolution::Tuple{Int64, Int64}` : video resolution (e.g., (1920,1080)).
"""
function showLayedOutVideo(traj::Vector{VS.VSTrajectory{VS.spatial}}, axes_layout; O::Union{GeometricalObject,Nothing} = nothing,
    column_widths = nothing, desired_pose::Vector{T} where T <: Real = [0], filename::Union{String,Nothing} = nothing, final_time::Real = 0, initial_time::Real = 0., 
    framerate::Integer = 30, interpolation_factor::Integer = 1, object_parameters::NamedTuple = NamedTuple(),
    live_video::Bool = false, logarithmic_time::Bool = false, row_widths = nothing, show_tracks_in_3D::Bool = true, show_tracks_in_screen::Bool = true, 
    theme::String = "", title::Union{String,GLM.Makie.LaTeXStrings.LaTeXString} = "", video_duration::Real = 30, video_resolution::Tuple{Int64, Int64} = (1920,1080))

    #=======

    General setup (definition of colors and theme, figures creation, etc.)

    =======#

    camera_axes_number = sum(1 for al in axes_layout if al.type == "camera");
    image_axes_number = length(axes_layout) - camera_axes_number;
    has_P = (isa(O,Points) || all((haskey(al,:O) && isa(al.O,Points)) for al in axes_layout if al.type == "image"));

    # unconsistency checks
    (image_axes_number != 0 && !has_P) && (@error "P matrix is needed to show the points on the screen.");

    time = GLM.Observable(Float64(initial_time));
    n = length(traj);
    
    # initialize theme and get points color
    pcolor = themeInitialization(theme);
    !haskey(object_parameters,:color) && (object_parameters = (object_parameters..., color = pcolor));

    # figure and axes creation
    fig = GLM.Figure(size = video_resolution);
    axes = Vector{Any}(undef,length(axes_layout));

    # define interpolated trajectories
    intTrs = map(i->TrajectoryInterpolator(traj[i].u,traj[i].t),1:n);
    
    tmax = (final_time == 0) ? maximum([traj[i].t[end] for i in eachindex(traj)]) : final_time;

    timestamps = logarithmic_time ? 
        map(x -> (x^2)*(tmax-time[]) + time[], range(0, 1, length = framerate*video_duration)) :
        range(time[], tmax, length = framerate*video_duration);

    cameras_list = Vector{Vector{GLM.Observable}}(undef,camera_axes_number);
    tracks_list  = Vector{Vector{GLM.Observable}}(undef,camera_axes_number);
    features_list = Vector{Vector{GLM.Observable}}(undef,image_axes_number);
    features_tracks_list = Vector{Vector{Vector{GLM.Observable}}}(undef,image_axes_number);
    P_sizes = Vector{Int}(undef, image_axes_number);
    points_order = Vector{Vector{Int}}(undef, image_axes_number);
    tracks_numbers = Vector{Vector{Int}}(undef, camera_axes_number);
    features_tracks_numbers = Vector{Vector{Int}}(undef, image_axes_number);
    kc = 1;
    ki = 1;

    general_settings = Dict{Symbol,Any}();
    all(!haskey(al.settings, :desired_pose) for al in axes_layout) && (general_settings[:desired_pose] = desired_pose);

    for (i,al) in enumerate(axes_layout)
        println("Elaborating video $i...")
        local_O = haskey(al,:O) ? al.O : O;

        if al.type == "camera"
            # create axes and get observables
            # NOTA BENE: in 3D, it's required to initialize title at creation, it is a known bug in GLMakie!
            axes[i] = haskey(al.settings,:title) ? GLM.Axis3(fig[al.axes_position...],aspect = :data, title = al.settings[:title]) : GLM.Axis3(fig[al.axes_position...],aspect = :data);
            tracks_numbers[kc] = al.list;
            cameras_list[kc],tracks_list[kc] = initializeCameraVideo!(axes[i], time, view(intTrs,al.list), timestamps, O = local_O,
            object_parameters = object_parameters; al.settings..., general_settings...);
            kc += 1;
        else
            # create axes and get observables
            axes[i] = GLM.Axis(fig[al.axes_position...], aspect = GLM.DataAspect(), yreversed = true);
            features_list[ki],features_tracks_list[ki] = initializeImageVideo!(axes[i], time, view(intTrs,al.list), timestamps, O = local_O,
            object_parameters = object_parameters; al.settings..., general_settings...);
            GLM.hidedecorations!(axes[i]);
            P_sizes[ki] = size(local_O,2);
            points_order[ki] = (haskey(object_parameters,:lines_order) && !isa(object_parameters.lines_order,Nothing)) ? map(i->findfirst(e->e==i,lines_order),1:P_sizes[ki]) : [1:P_sizes[ki]...]; # "re-order" points order from show_lines; used to properly define points' tracks in the screen (below)
            ki += 1;
        end
    end

    println("Done.")

    #=======

    Initialize video stuff

    =======#

    obs_title = GLM.lift(time) do t
        if title == ""
            return "t = $(round(t, digits = 1)) s";
        else
            return GLM.L"%$title - time: $%$(round(t, digits = 1))\,\mathrm{s}$";
        end
    end

    GLM.Label(fig[0,1:end], obs_title, font=:bold, fontsize=20);
    
    if !isa(column_widths,Nothing)
        for i in eachindex(column_widths)
            GLM.colsize!(fig.layout,i,GLM.Relative(column_widths[i]));
        end
    end

    if !isa(row_widths,Nothing)
        for i in eachindex(row_widths)
            GLM.rowsize!(fig.layout,i,GLM.Relative(row_widths[i]));
        end
    end

    # filename
    filename = isa(filename,Nothing) ? "videos/spatial_"*Dates.format(Dates.now(), "yyyymmdd-HH:MM")*".mp4" : "videos/"*filename*".mp4";
    mkpath(dirname(filename));
    
    #=======

    Video plotting

    =======#

    if live_video
        display(fig);

        for t in timestamps
            time[] = t;
            if show_tracks_in_3D
                for (il,tracks) in enumerate(tracks_list)
                    for it in eachindex(tracks)
                        four_vertices = cameras_list[il][it][][1:4,:];
                        new_point = sum(map(i->four_vertices[i,:],1:4))/4;
                        #=println("""
                        DEBUG:
                            it: $it 
                            new_point: $new_point
                            tracks[it]: $(tracks[it][])
                        """)=#
                        push!(tracks[it][], GLM.Point3f( new_point ));
                        notify(tracks[it]);
                    end
                end
            end
            if show_tracks_in_screen
                for (il,screen_tracks) in enumerate(features_tracks_list)
                    for it in eachindex(screen_tracks)
                        for j = 1:P_sizes[il]
                            push!(screen_tracks[it][j][], features_list[il][it][][points_order[il][j]]);
                            notify(screen_tracks[it][j]);
                        end
                    end
                end
            end
            sleep(1/(1.5*framerate));
        end
    else
        # video creation
        GLM.record(fig, filename, timestamps;
                framerate = framerate) do t
            
            finer_time = range(time[],t,interpolation_factor+1)[2:end];
            
            if show_tracks_in_3D
                for (il,tracks) in enumerate(tracks_list)
                    for it in eachindex(tracks)
                        new_points = [GLM.Point3f((intTrs[tracks_numbers[il][it]])(tf)) for tf in finer_time];
                        append!(tracks[it][], new_points);
                        notify(tracks[it]);
                    end
                end
            end
            time[] = t;
            if show_tracks_in_screen
                for (il,screen_tracks) in enumerate(features_tracks_list)
                    for it in eachindex(screen_tracks)
                        for j = 1:P_sizes[il]
                            push!(screen_tracks[it][j][], features_list[il][it][][points_order[il][j]]);
                            notify(screen_tracks[it][j]);
                        end
                    end
                end
            end
        end
    end
    
end

function initializeCameraVideo!(ax, time, intTrs, timestamps; O::Union{GeometricalObject,Nothing} = nothing, axes_limits::Vector{T} where T<:Real = [0], azimuth = nothing,
    colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, color_gradient::Bool = false, desired_pose::Vector{T} where T <: Real = [0],
    elevation = nothing, object_parameters::NamedTuple = NamedTuple(), scale::Real=4, static_colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing,
    static_poses::Vector{Vector{T}} where T <: Real = [[0]], style::Union{T, Vector{T}} where T <: Union{Symbol, Vector{N} where N<:Real} = :solid,
    title::Union{Nothing,String,GLM.Makie.LaTeXStrings.LaTeXString} = nothing)

    #=======

    General setup (definition of colors and theme, figures creation, etc.)

    =======#

    n = length(intTrs);
    
    # title, colors and styles initialization
    !isa(title,Nothing) && (ax.title = title);
    colors = colorsInitialization(colors, n, color_gradient);
    style = styleInitialization(style,n);

    (static_poses != [[0]]) && (static_colors = colorsInitialization(static_colors,length(static_poses)));

    #=======

    Automatic resizing of axes limits, based on trajectories largest values

    =======#

    # the following manages the automatic resizing of the axes
    if axes_limits == [0]
        limits = [Inf,-Inf,Inf,-Inf,Inf,-Inf];

        for tr in intTrs
            for t in timestamps
                tmp=tr(t);

                for i in 1:3
                    limits[2*i-1:2*i] .= (min(limits[2*i-1],tmp[i]),max(limits[2*i],tmp[i]));
                end
            end

            tr(tr.t[end]+1); # reset trajectory's counter; TrajectoryInterpolator is made for sequential access, see Plotter.jl for implementation
        end

        cameraScale = scale*maximum([limits[i+1]-limits[i] for i in [1,3,5]])/100;
        limits += [-2*cameraScale,2*cameraScale,-2*cameraScale,2*cameraScale,-2*cameraScale,2*cameraScale]; # add a bit of padding around the plot
    else
        limits = copy(axes_limits);
        cameraScale = scale*maximum([limits[i+1]-limits[i] for i in [1,3,5]])/(100+4*scale);
    end
        
    GLM.limits!(ax,limits...);
    !isa(azimuth,Nothing) && (ax.azimuth = azimuth);
    !isa(elevation,Nothing) && (ax.elevation = elevation);

    #=======
    
    Plot static stuff
    
    =======#

    !isa(O, Nothing) && (plotObject!(ax,O,object_parameters));

    # camera shape definition
    v,f,sc = getCameraShape();

    R(x) = RC.quaternionToRotationMatrix(x[4:end]);
    p(x) = x[1:3];    

    if desired_pose != [0]
        GLM.poly!(ax, transpose((R(desired_pose)*transpose(v).*cameraScale).+p(desired_pose)), f, 
        strokewidth = 2.5, linestyle = :dash, color = GLM.RGBA{Float32}(51/255,100/255,82/255,0.3f0), strokecolor = sc);
    end

    if static_poses != [[0]]
        for (i,pose) in enumerate(static_poses)
            GLM.poly!(ax, transpose((R(pose)*transpose(v).*cameraScale).+p(pose)), f, 
            strokewidth = 2.5, linestyle = :dash, color = static_colors[i], strokecolor = sc);
        end
    end

    #=======

    Initialize video stuff

    =======#

    # cameras and tracks "objects", comments below are "element-wise"
    cameras = Vector{GLM.Observable}(undef,n); # matrix, depending on time; each column is a vertex of the camera shape
    tracks = Vector{GLM.Observable}(undef,n); # list of 3D points, a point is added after each time-step; each point is the camera's position at a given time instant

    for i in eachindex(intTrs)
        cameras[i] = GLM.@lift transpose((R(intTrs[i]($time))*transpose(v).*cameraScale).+p(intTrs[i]($time)));
        tracks[i] = GLM.Observable([GLM.Point3f(intTrs[i](time[]))]);

        GLM.poly!(ax, cameras[i], f, strokewidth = 2.5, color = colors[i], strokecolor = sc);
        GLM.lines!(ax, tracks[i], linewidth = 2, color = colors[i], linestyle = style[i]);
    end

    return cameras,tracks;
    
end

function initializeImageVideo!(ax_screen, time, intTrs, timestamps;
    O::Points, colors::Union{T,Vector{T},Nothing} where T<:Union{GLM.Color,GLM.ColorAlpha,Symbol} = nothing, color_gradient::Bool = false, 
    desired_pose::Vector{T} where T <: Real = [0], object_parameters::NamedTuple = NamedTuple(), screen_limits::Vector{T} where T<:Real = [0],
    style::Union{T, Vector{T}} where T <: Union{Symbol, Vector{N} where N<:Real} = :solid, title::Union{Nothing,String,GLM.Makie.LaTeXStrings.LaTeXString} = nothing)

    #=======

    General setup (definition of colors and theme, figures creation, etc.)

    =======#

    P = O.P;
    lines_order = haskey(object_parameters,:lines_order) ? object_parameters.lines_order : nothing;
    lines_thickness = haskey(object_parameters,:lines_thickness) ? object_parameters.lines_thickness : 2;
    point_size = haskey(object_parameters,:point_size) ? object_parameters.point_size : 20;

    show_lines = !isa(lines_order,Nothing);
    points_order = show_lines ? map(i->findfirst(e->e==i,lines_order),1:size(P,2)) : [1:size(P,2)...]; # "re-order" points order from show_lines; used to properly define points' tracks in the screen (below)

    # features evaluation, both in form of a matrix or a list of 2D points
    s(x,P) = (Pc = transpose(RC.quaternionToRotationMatrix(x[4:end]))*(P.-x[1:3]); return [Pc[1,:]./Pc[3,:], Pc[2,:]./Pc[3,:]]);
    s_p(x,P) = (Pc = transpose(RC.quaternionToRotationMatrix(x[4:end]))*(P.-x[1:3]); return [GLM.Point2f(x,y) for (x,y) in zip(Pc[1,:]./Pc[3,:], Pc[2,:]./Pc[3,:])]);

    n = length(intTrs);
    
    # title, colors and styles initialization
    !isa(title,Nothing) && (ax_screen.title = title);
    colors = colorsInitialization(colors, n, color_gradient);
    style = styleInitialization(style,n);

    #=======

    Automatic resizing of axes limits, based on trajectories largest values

    =======#

    # the following if manages the automatic resizing of the axes
    # it is badly structured, but it manages to read all trajectories only once
    if screen_limits == [0]
        c_limits = [Inf,-Inf,Inf,-Inf];

        for tr in intTrs
            for t in timestamps
                tmp=tr(t);
                s_tmp = s(tmp,P);

                for i in 1:2
                    c_limits[2*i-1:2*i] .= (minimum([s_tmp[i]...,c_limits[2*i-1]]),maximum([s_tmp[i]...,c_limits[2*i]]));
                end
            end

            tr(tr.t[end]+1); # reset trajectory's counter; TrajectoryInterpolator is made for sequential access, see Plotter.jl for implementation
        end

        c_widths = 0.01*[c_limits[i+1]-c_limits[i] for i in [1,3]];
        c_limits += [-c_widths[1],c_widths[1],-c_widths[2],c_widths[2]];

    else
        c_limits = copy(screen_limits);
    end
        
    GLM.limits!(ax_screen,c_limits...); # inline if!

    #=======
    
    Plot static stuff
    
    =======#

    if desired_pose != [0]
        s_tmp = s(desired_pose,P);
        if !show_lines
            GLM.scatter!(ax_screen, s_tmp[1], s_tmp[2], marker = :cross,
            markersize = point_size+2, color = GLM.RGBA{Float32}(51/255,100/255,82/255,0.8f0));
        else
            GLM.lines!(ax_screen, s_tmp[1][lines_order], s_tmp[2][lines_order], color =  GLM.RGBA{Float32}(51/255,100/255,82/255,0.75f0), linestyle = :dash, linewidth = lines_thickness);
        end
    end

    #=======

    Initialize video stuff

    =======#

    features = Vector{GLM.Observable}(undef,n); # list of N 2D points, depending on time; each point is one of the tracking points' features
    screen_tracks = Vector{Vector{GLM.Observable}}(undef,n); # vector of obs. of length N, each element is a list of 2D points that increases at each time-step

    for i in eachindex(intTrs)
        if !show_lines
            features[i] = GLM.@lift s_p(intTrs[i]($time),P);
            GLM.scatter!(ax_screen, features[i], color = colors[i], markersize = point_size);
        else
            features[i] = GLM.@lift s_p(intTrs[i]($time),P)[lines_order];
            GLM.lines!(ax_screen, features[i], color = colors[i], linewidth = lines_thickness, linestyle = style[i]);
        end
        screen_tracks[i] = Vector{GLM.Observable}(undef,size(P,2));

        for j in 1:size(P,2)
            screen_tracks[i][j] = GLM.Observable([features[i][][points_order[j]]]);
            GLM.lines!(ax_screen, screen_tracks[i][j], linewidth = 1, color = colors[i], linestyle = :dash);
        end            
    end

    return features,screen_tracks;
end
