using InfiniteOpt, Ipopt, Plots
#include("vehicleParameters/carParametersModule.jl")
#using .carParametersModule



# some parameters and initial conditions:
tfGuess = 1
trackLength = 75
m = 1
    # initial conditions
    x0 = [0,0,0]
    v0 = [0,0,0]
    s0 = 0




## HERE STARTS THE INFINITE OPT STUFF!!!!!
model = InfiniteModel(optimizer_with_attributes(Ipopt.Optimizer, "print_level" => 0));


# Problem Variables
@infinite_parameter(model, t in [0, tfGuess], num_supports = 200)


@variables(model, begin
    # track length
    s, Infinite(t)
    # state variables
    x[1:3], Infinite(t)
    v[1:3], Infinite(t)
    V, Infinite(t)      # Velocity in TNB Frame
    #β, Infinite(t)
    ψ, Infinite(t)      # Angle to TNB Frame
    ω, Infinite(t)      # Yaw rate

    # control variables
    -1 <= u[1:2] <= 1, Infinite(t)
end)

@variable(model, tf >= 0, start = tfGuess)

# TNB Frame Stuff, attempt 1, straight line
        #=
            @parameter_function(model, T == (s) -> [1;0;0])
            @parameter_function(model, N == (s) -> [0;1;0])
            @parameter_function(model, B == (s) -> [0;0;1])

            @parameter_function(model, κ == (s) -> 0)
            @parameter_function(model, τ == (s) -> 0)
        =#
        # didn't work, try more basic for now:
        T = [1;0;0]
        N = [0;1;0]
        B = [0;0;1]
        κ = .05
        τ = 0
## 
# functions, test for control variables. 
throttleMap(u,v) = 1.2u - .3v
brakeMap(u) = .5u

throttleBrake(u,v) = InfiniteOpt.ifelse(u ≥ 0, throttleMap(u,v), brakeMap(u))
maxTractionAvailable(v) = .3 + .1v^2

getV(vel,angle) = vel/cos(angle)
#@register(model,getV(vel))

## Objective Function! minimize final time
@objective(model,Min,tf)

# Initial Conditions
    @constraint(model, [i = 1:3], x[i](0) == x0[i])
    @constraint(model, [i = 1:3], v[i](0) == v0[i])
    @constraint(model, s(0) == s0)
    @constraint(model, ψ(0) == 0)
    @constraint(model, ω(0) == 0)
    # Make it go! s at final time is at the end of track (s(t=tf) = s_f)
    @constraint(model, s(tfGuess) == trackLength)

# Dynamic Constraints
    # Dynamics in TNB Frame: hopefully this takes care of most of the Dynamics
            # also will need a basis for normal force, don't want to un-generalize it and say it's always
            # in the binormal direction. This would make it more sine-cosiney, should probably look more
            # into rotations
    @constraint(model, m*(∂(v[1], t) - κ*v[1]v[2]) == (tf/tfGuess)* throttleBrake(u[1], V) * cos(ψ))
    @constraint(model, m*(∂(v[2], t) + κ*v[1]^2)   == (tf/tfGuess)* throttleBrake(u[1], V) * sin(ψ))

    # For now, much simplified velocity & yaw control 
    @constraint(model, ∂(ω, t) == (tf/tfGuess) * u[2])

    # update position state variables in TNB Ref Frame
        # only update x2,x3. x1 is always zero, as the TNB frame moves along the track
        # with the car, for now
    @constraint(model, [i = 2:3], ∂(x[i], t) == (tf/tfGuess) * v[i])
    
    @constraint(model, ∂(ψ, t) == (tf/tfGuess) * ω)

    # TNB Movement: update s by v1
    @constraint(model, ∂(s,t) == (tf/tfGuess) * v[1])

    # Car Velocity from velocity components
        # super weird, does not work with a norm
        # using v1/cosψ
    @constraint(model,V == getV(v[1],ψ))

    
    #@constraint(model, ∂(V, t) == (tf/tfGuess) * throttleBrake(u[1],V))



# limit track width!
@constraint(model, -2 <= x[2] <= 2)

# try to enforce some control vector continuity stuff
    # doesn't converge without this guy, which makes sense sorta
    @constraint(model, u[1](0) >= 0)
    @constraint(model, u[2] == 0)
    
    # these guys, eh, seems right. v3=0 for now, should be faster?
    @constraint(model, v[1] >= 0)
    @constraint(model, v[3] == 0)


# traction constraint, pls work!!!!
@constraint(model, [i = 1:2], u[i] <= maxTractionAvailable(V))
        # it worky! sorta, max traction not really well related to yaw rate..

# linalg go brrrrrrr
optimize!(model)




# plots and plotts and plottts
x_opt = value.(x);
v_opt = value.(v);

using Plots
#scatter(xw[1,:], xw[2,:], label = "Waypoints")
#p3 = plot!(x_opt[1], x_opt[2], label = "Trajectory")
#xlabel!("x_1")
#ylabel!("x_2")



    p1 = Plots.plot(value.(s),x_opt[2]; ylabel = "X-Position")
    p2 = Plots.plot(v_opt[1]; ylabel = "X-Velocity")
    
    p4 = Plots.plot(value.(u); ylabel = "control")
    

    display(plot(p1, p2, p4, layout=(2,2), legend=false))

    print(solution_summary(optimizer_model(model)))

    print("Optimal final time is $(value.(tf)) seconds")