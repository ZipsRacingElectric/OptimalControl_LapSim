using InfiniteOpt, Ipopt, Plots

xw = [1 4 6 1; 1 3 0 1] # positions
tw = [0, 25, 50, 60];    # times

m = InfiniteModel(optimizer_with_attributes(Ipopt.Optimizer, "print_level" => 0));


finalTimeInitialGuess = 60


@infinite_parameter(m, t in [0, finalTimeInitialGuess], num_supports = 61)

@variables(m, begin
    # state variables
    x[1:2], Infinite(t)
    v[1:2], Infinite(t)
    # control variables
    -1 <= u[1:2] <= 1, Infinite(t), (start = 0)
end)

@variable(m, 0 <= tf, start = 2)



#@objective(m, Min, ∫(u[1]^2 + u[2]^2, t))
@objective(m,Min,tf)

# Initial Conditions
@constraint(m, [i = 1:2], v[i](0) == 0)

# Dynamic Constraints
@constraint(m, [i = 1:2], ∂(x[i], t) == (tf/finalTimeInitialGuess) * v[i])
@constraint(m, [i = 1:2], ∂(v[i], t) == (tf/finalTimeInitialGuess) * u[i])

# hit waypoints
@constraint(m, [i = 1:2, j = eachindex(tw)], x[i](tw[j]) == xw[i, j])



# linalg go brrrrrrr
optimize!(m)

x_opt = value.(x);

using Plots
scatter(xw[1,:], xw[2,:], label = "Waypoints")
p3 = plot!(x_opt[1], x_opt[2], label = "Trajectory")
xlabel!("x_1")
ylabel!("x_2")



    p1 = Plots.plot(x_opt[1]; ylabel = "X-Position")
    p2 = Plots.plot(x_opt[2]; ylabel = "Y-Position")
    
    p4 = Plots.plot(value.(u); ylabel = "control")
    

    display(plot(p1, p2, p3, p4, layout=(2,2), legend=false))

    print(solution_summary(optimizer_model(m)))

    print("Optimal final time is $(value.(tf)) seconds")