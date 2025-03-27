using InfiniteOpt, Plots




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
    n = 4
    
    p1 = Plots.plot(runData.t, runData.u; ylabel = "Control Input")
    p2 = Plots.plot(runData.t, runData.v; ylabel = "Velocities")
    p3 = Plots.plot(runData.t, runData.V; ylabel = "Speed")
    p4 = Plots.plot(runData.t, runData.ψ; ylabel = "Angle to TNB, ψ")
        
    display(plot(p1, p2, p3, p4, layout=(n,1), legend=false))
end

function plotPosition(runData)
    # Still needs TNB stuff, shouldnt be too hard
    p1 = Plots.plot(runData.s, runData.x[2]; xlabel = "x",ylabel = "y")

    display(plot(p1,layout=(1,1), legend=false))


end