module Plotter

    import Plots
    import GLMakie as GLM
    import LinearAlgebra as LA
    import Dates

    import ..Desiderata as Des
    import ..RotationConversions as RC
    import ..VSSystems as VS
    import ..Utilities as Uti
    include("Plotter_geometricalobjects.jl")
    include("Plotter_planar.jl")
    include("Plotter_circular.jl")
    include("Plotter_spatial.jl")
    
    export animateTrajectories,plotCameraTrajectories,plotCameraTrajectories!,plotCameras,plotCameras!,plotImageTrajectories,plotImageTrajectories!,plotScreen,plotScreen!,showLayedOutVideo,showTrajectoryVideo,
           Cylinder,Points

    #=for n in names(@__MODULE__; all=true)
        if Base.isidentifier(n) && n ∉ (Symbol(@__MODULE__), :eval, :include)
            @eval export $n
        end
    end=#

    struct TrajectoryInterpolator
        u::Vector{Vector{Float64}}
        t::Vector{Float64}
        i::Vector{Int} # one-element vector; an Int would've been immutable!
        TrajectoryInterpolator(u,t) = new(u,t,[1]);
    end

    function (T::TrajectoryInterpolator)(τ::Real)
        if τ < T.t[1]
            @error "The requested time instant is before the start of the simulation.";
        end
        if τ >= T.t[end]
            T.i[1] = 1;
            return T.u[end];
        end
    
        i = T.i[1];
        while T.t[i] <= τ
            i += 1;
        end
        
        t0 = T.t[i-1];
        t1 = T.t[i];
        
        T.i[1] = i;
    
        return ((t1-τ)*T.u[i-1] + (τ-t0)*T.u[i])./(t1-t0);
    end

    function getColorShades(N::Integer)
        if N <= 0
            @error "Cannot provide a negative number of colors.";
        end
        if N < 4
            return [GLM.RGBA{Float32}(1.0f0,0.0f0,0.0f0,1.0f0),GLM.RGBA{Float32}(0.0f0,1.0f0,0.0f0,1.0f0),
            GLM.RGBA{Float32}(0.0f0,0.0f0,1.0f0,1.0f0)][1:N];
        end
        
        offset = [0,0,0,0];
        ord = [4,2,3];
    
        srange(i,e,l) = (l != 1) ? range(i,e,length=l) : [i]; # generalizes range to accept length = 1

        # offset says where to put the "additional colors" when N is not divisible by 4 
        for i in 1:(N % 4)
            offset[ord[i]] += 1;
        end
    
        L = [N÷4 for i in 1:4] + offset;
    
        R2B = [GLM.RGBA{Float32}((1-e),0.0f0,e,1.0f0) for e in srange(0, 1, L[1])];
        B2G = [GLM.RGBA{Float32}(0.0f0,e,(1-e),1.0f0) for e in srange(0, 1, L[2])];
        G2Y = [GLM.RGBA{Float32}(e,1.0f0,0.0f0,1.0f0) for e in srange(0, 1, L[3])];
        Y2R = [GLM.RGBA{Float32}(1.0f0,(1-e),0.0f0,1.0f0) for e in srange(0, 1, L[4])];
    
        return [R2B...,B2G...,G2Y...,Y2R...];

        # the following function provides better functionalities with one line; currently not deployed because of retrocompatibility, 
        # it will become official once the paper is submitted 
        #function getColorShades(N::Integer;S::Real=85,V::Real=85)
        #return map(e->GLM.HSV{Float64}(e,S/100,V/100),range(0,360,length=N+1)[1:end-1]);
        #end
    end

end