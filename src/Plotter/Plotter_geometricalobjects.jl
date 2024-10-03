#=========================================================================
Motivation: In visual servoing, we often need to get our visual features
from an object which isn't simply a set of markers.
We define in this file a set of geometrical objects of interest, together
with a way to plot each of them.
=========================================================================#


#==================
Objects definitions
==================#

abstract type GeometricalObject end;

struct Cylinder <: GeometricalObject
    a::Vector{T} where T<:Real
    P::Vector{T} where T<:Real
    R::T where T<:Real
    Cylinder(a::Vector{T} where T<:Real, P::Vector{T} where T<:Real, R::T where T<:Real) = new(a,P,R);
    Cylinder(v::Vector{T} where T<:Real) = new(v[1:3],v[4:6],v[7]);
end
    
struct Points <: GeometricalObject
    P::Matrix{T} where T<:Real
    Points(P::Matrix{T} where T<:Real) = new(P);
    Points(P::Vector{Vector{T}} where T<:Real) = new(hcat(P...));
end

# Overload getindex for Points to forward indexing to the internal matrix P
function Base.getindex(p::Points, inds...)
    return getindex(p.P,inds...);
end

# Overload size for Points to return the size of the internal matrix P
function Base.size(p::Points, dim::Int=0)
    if dim == 0
        return size(p.P)
    else
        return size(p.P, dim)
    end
end

# Overload axes for Points to return the axes of the internal matrix P
function Base.axes(p::Points, dim::Int=0)
    if dim == 0
        return axes(p.P)
    else
        return axes(p.P, dim)
    end
end

#===============
Objects plotting
===============#

function plotObject(O::GeometricalObject, parameters::NamedTuple = NamedTuple())
    fig = Figure();
    ax = Axis3(fig[1,1], aspect = :data);    

    plotObject!(ax,O,parameters);
    return fig;
end

function plotObject!(ax, C::Cylinder, parameters::NamedTuple = NamedTuple())
    a,P,R = C.a, C.P, C.R;

    N     = haskey(parameters,:N)     ? parameters.N : 40;
    color = haskey(parameters,:color) ? parameters.color : :black;
    lower_face = haskey(parameters,:lower_face) ? parameters.lower_face : -2;
    upper_face = haskey(parameters,:upper_face) ? parameters.upper_face : 2;

    if abs(transpose(a)*a - 1) >= 1e-8 || abs(transpose(a)*P) >= 1e-8 || R <= 0
        @error "Invalid cylinder parameters."
        return;
    end

    b,c = eachcol(LA.nullspace(transpose(a)));
    p = [R*(b * cos(θ) + c * sin(θ)) for θ in range(0,2*π,length=N)];

    l = Vector{GLM.Point3f}(undef,3*N);

    P1 = hcat(p...) .+ (P + upper_face*a);
    P2 = hcat(p...) .+ (P + lower_face*a);
    P3 = hcat(p...) .+ (P + (upper_face+lower_face)*a/2);

    for i in 1:N
        l[3*i-2:3*i] = GLM.Point3f.([P1[:,i],P2[:,i],NaN]);
    end

    GLM.lines!(ax,[GLM.Point3f.(eachcol(P1))...,GLM.Point3f.(eachcol(P2))...,GLM.Point3f.(eachcol(P3))...,l...], color = color)
end

function plotObject!(ax, P::Points, parameters::NamedTuple = NamedTuple())
    lines_thickness = haskey(parameters,:lines_thickness) ? parameters.lines_thickness : 2;
    point_size      = haskey(parameters,:point_size)      ? parameters.point_size : 20;
    color           = haskey(parameters,:color)           ? parameters.color : :black;
    show_lines      = haskey(parameters,:lines_order) && !isa(parameters.lines_order, Nothing);
    show_lines && (lines_order = parameters.lines_order);

    if !show_lines
        GLM.scatter!(ax,P[1,:], P[2,:], P[3,:], markersize=point_size,color=color); # plots tracking points
    else
        GLM.lines!(ax, P[1,:][lines_order], P[2,:][lines_order], P[3,:][lines_order], color = color, linewidth = lines_thickness);
    end
end