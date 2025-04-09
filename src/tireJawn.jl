using LinearAlgebra


#x0 = [π/2, 4, 0]; # x(0) for x1,x2,x3   

#m = InfiniteModel(Ipopt.Optimizer);


function shittyTires(sa) 
    return -20((sa*180/π)-10)^2 + 2000

end



#function Fy(sl,sa,Fz)

#end
#=
breaks = collect(-0.3:0.05:0.3)  # 13 breakpoints → 12 pieces
coefs = rand(12, 4)  # just an example: 12 segments, 4 coeffs per piece (cubic)

model = InfiniteModel()

@infinite_parameter(model, α ∈ [-0.3, 0.3])
@variable(model, Fy, Infinite(α))



function eval_piecewise_pp(α, breaks, coefs)
    segments = length(breaks) - 1
    return sum(
        ((α ≥ breaks[i]) * (α ≤ breaks[i+1])) *
        dot(coefs[i, :], [(α - breaks[i])^3, (α - breaks[i])^2, (α - breaks[i]), 1])
        for i in 1:segments
    )
end


@constraint(model, Fy == eval_piecewise_pp(α, breaks, coefs))
jawn = @expression(model,Fy^2)

@objective(model,Max,jawn)

=#