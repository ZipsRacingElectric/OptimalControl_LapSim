using InfiniteOpt, Ipopt, Plots

finalTimeInitialGuess = 1

xw = [1 75; 0 0] # positions
tw = [0, finalTimeInitialGuess];    # times

m = InfiniteModel(optimizer_with_attributes(Ipopt.Optimizer, "print_level" => 0));




# Problem Variables
@infinite_parameter(m, t in [0, finalTimeInitialGuess], num_supports = 101)
@variables(m, begin
    # state variables
    x[1:2], Infinite(t)
    v[1:2], Infinite(t)
    # control variables
    -1 <= u[1:2] <= 1, Infinite(t)
end)
@variable(m, 0 <= tf, start = finalTimeInitialGuess)

# functions, test for control variables. Should work only if function f = f(u(t)) only,
    # I think f(u(t), x(t)) means diff eq's are wrong

    # eh not actually sure anymore, f(u,v) seems to work
throttleMap(u,v) = 1.2u - .3v
brakeMap(u) = .5u

throttleBrake(u,v) = InfiniteOpt.ifelse(u ≥ 0, throttleMap(u,v), brakeMap(u))
    
maxTractionAvailable(v) = .3 + .1v^2

#@objective(m, Min, ∫(u[1]^2 + u[2]^2, t))
@objective(m,Min,tf)

# Initial Conditions
@constraint(m, [i = 1:2], v[i](0) == 0)

# Dynamic Constraints
@constraint(m, [i = 1:2], ∂(x[i], t) == (tf/finalTimeInitialGuess) * v[i])
@constraint(m, [i = 1:2], ∂(v[i], t) == (tf/finalTimeInitialGuess) * throttleBrake(u[i],v[i]))



# hit waypoints
@constraint(m, [i = 1:2, j = eachindex(tw)], x[i](tw[j]) == xw[i, j])

# try to enforce some control vector continuity stuff
    # doesn't converge without this guy, which makes sense sorta
@constraint(m, u[1](0) >= 0)
@constraint(m, u[2] == 0)


# traction constraint, pls work!!!!
@constraint(m, [i = 1:2], u[i] <= maxTractionAvailable(v[1]))

# linalg go brrrrrrr
optimize!(m)

x_opt = value.(x);
v_opt = value.(v);

using Plots
scatter(xw[1,:], xw[2,:], label = "Waypoints")
p3 = plot!(x_opt[1], x_opt[2], label = "Trajectory")
xlabel!("x_1")
ylabel!("x_2")



    p1 = Plots.plot(x_opt[1]; ylabel = "X-Position")
    p2 = Plots.plot(v_opt[1]; ylabel = "X-Velocity")
    
    p4 = Plots.plot(value.(u); ylabel = "control")
    

    display(plot(p1, p2, p3, p4, layout=(2,2), legend=false))

    print(solution_summary(optimizer_model(m)))

    print("Optimal final time is $(value.(tf)) seconds")