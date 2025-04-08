using InfiniteOpt, Plots, LinearAlgebra




function getParameterNames(m::InfiniteModel)
    if has_values(m)
        n = m.infinite_vars.last_index
        print("$(n) total infinite variables")

        symbolNames = Vector{Symbol}(undef, n)
        for i = 1:n
            symbolNames[i] = Symbol(model.infinite_vars.vector[i].name)
        end

        return(symbolNames)
    else print("Model has not been solved or has no values, can't get parameter names, dummy!") end

end


function getCenterline(runData,T,N,B,κ,τ)

    for i in 1:length(rundata.s)-1

        

    end

end

function plotTimeHistories(runData)
    n = 5
    
    p1 = Plots.plot(runData.t, runData.u; ylabel = "Control Input")
    p2 = Plots.plot(runData.t, runData.v; ylabel = "Velocities")
    p3 = Plots.plot(runData.t, runData.V; ylabel = "Speed")
    p4 = Plots.plot(runData.t, runData.a; ylabel = "Acceleration")
    p5 = Plots.plot(runData.t, runData.ψ; ylabel = "Angle to TNB, ψ")
        
    display(plot(p1, p2, p3, p4, p5, layout=(n,1)))
end

function plotPosition(runData)
    # Still needs TNB stuff, shouldnt be too hard

    # Initialize the Frenet-Serret frame
    T = zeros(length(runData.s), 3)
    N = zeros(length(runData.s), 3)
    B = zeros(length(runData.s), 3)

    T[1, :] = [1, 0, 0]
    N[1, :] = [0, 1, 0]
    B[1, :] = [0, 0, 1]

    s = range(0, runData.s[end], length=length(lapRunData.s))
    kappa = runData.κ
    x_track = zeros(length(runData.s), 3)
    x_car = zeros(length(runData.s), 3)

    makeUnitVector(v) = v./norm(v)
    frenetSerret(κ,τ,T,N,B) = [0 κ 0;-κ 0 τ; 0 -τ 0]*[T';N';B']

    for i in 2:length(s)
        TNB_prime = frenetSerret(kappa[i], 0, T[i-1, :], N[i-1, :], B[i-1, :])
    
        # Can't assume this stays constant, different discretization schemes might use variable spacing
        # oh and velocity changes, and s is parameterized now by t, so spacing not even
        delta_s = s[i] - s[i-1]
    
        T[i, :] = makeUnitVector(T[i-1, :] + TNB_prime[1, 1:3] * delta_s)
        N[i, :] = makeUnitVector(N[i-1, :] + TNB_prime[2, 1:3] * delta_s)
        B[i, :] = makeUnitVector(B[i-1, :] + TNB_prime[3, 1:3] * delta_s)
    
        x_track[i, :] = x_track[i-1, :] + delta_s .* T[i, :]
        x_car[i, :] = x_track[i, :]
    end
    x_car = x_car + lapRunData.x[2] .* N 
    


    p1 = Plots.plot(x_track[:,1], x_track[:,2], xlabel = "x",ylabel = "y")
    plot!(p1,x_car[:,1],x_car[:,2])

    display(plot(p1,layout=(1,1)))


end