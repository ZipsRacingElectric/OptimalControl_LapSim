using InfiniteOpt, Ipopt, Plots
include("vehicleParameters/carParametersModule.jl")
using .CarParametersModule
include("modelUtilities.jl")


# Car Spreadsheet and Parameters!
    # gotta update this to look in github, and not have path that only works for me 🤭
spreadsheetPath = "C:\\Users\\benmo\\Documents\\GitHub\\Vehicle-Dynamics\\MATLAB\\vehicle_data\\zr25_data.xlsx"
#zr25 = CarParametersModule.create_car(spreadsheetPath)

    # commented out so it doesn't run every time, theres probably a better way to architect this,..

    # unfortunately, most of the parameters I want to pull from this, will be better to include
    # as finite parameters in the infiniteOpt model, so cant use directly, will have dumb stuff
    # like @parameter(model, m, zr25.mass) or something. This is to allow more human readable
    # equations in the model, rather than scalars, and one can supposedly perform some
    # sensitivity analysis, according to chatGPT (lol)

# some parameters and initial conditions:
tfGuess = 1
trackLength = 75

    # initial conditions
    x0 = [0,0,0]
    v0 = [0,0,0]
    s0 = 0




## HERE STARTS THE INFINITE OPT STUFF!!!!!
model = InfiniteModel(optimizer_with_attributes(Ipopt.Optimizer, "print_level" => 0));

# Car parameters, as finite parameters (see above)
@finite_parameter(model, m == zr25.mass_total)
@finite_parameter(model, l == zr25.wheelbase)
#@finite_parameter(model, a == zr25.a)
#@finite_parameter(model, b == zr25.b)
@finite_parameter(model, cgDist == zr25.front_mass_distribution)
@finite_parameter(model, CdA == (1.4+zr25.Cd*zr25.frontal_area))       # 1.6 lmao
@finite_parameter(model, ClA == zr25.Cl*zr25.frontal_area)
#@finite_parameter(model, r_wheel == zr25.tire_loaded_radius)
#@finite_parameter(model, n_gearbox == zr25.gear_ratio)
@finite_parameter(model, torque_max == 21*zr25.gear_ratio/zr25.tire_loaded_radius)
@finite_parameter(model, μ == zr25.tire_mu*zr25.tire_mu_correction_factor*.5)

@finite_parameter(model, g == zr25.g)
@finite_parameter(model, ρ == zr25.air_density)


# Problem Variables
@infinite_parameter(model, t in [0, tfGuess], num_supports = 200)


@variables(model, begin
    # track length
    s, Infinite(t)
    # state variables
    x[1:3], Infinite(t)
    v[1:3], Infinite(t)
    a[1:3], Infinite(t)
    V, Infinite(t)      # Velocity in TNB Frame
    #β, Infinite(t)
    ψ, Infinite(t)      # Angle to TNB Frame
    ω, Infinite(t)      # Yaw rate

    F_f[1:3], Infinite(t)
    F_r[1:3], Infinite(t)

    F_car_x, Infinite(t)
    F_car_y, Infinite(t)
    F_car_z, Infinite(t)

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
        κ = 0
        τ = 0
## 
# functions, test for control variables. 
throttleMap(u,v) = 4*(torque_max)u #-.3v
brakeMap(u) = .5u

throttleBrake(u,v) = InfiniteOpt.ifelse(u ≥ 0, throttleMap(u,v), brakeMap(u))

F_aero(coeff,vel) = .5ρ * coeff * vel^2
maxTractionAvailable(vel) = (.25*(m*g + .5ClA*ρ*vel^2)*μ) / torque_max

getV(vel,angle) = vel/cos(angle)

# Rotation matrix jawns- might be easier in axis angle formulation??
#rotate_z(θ_z) = [cos(θ_z), -sin(θ_z), 0; sin(θ_z), cos(θ_z), 0; 0, 0, 1]
#rotate_y(θ_y) = [cos(θ_y), 0, sin(θ_y); 0, 1, 0; -sin(θ_y), 0, cos(θ_y)]
#rotate_x(θ_x) = [1, 0, 0; 0, cos(θ_x), -sin(θ_x); 0, sin(θ_x), cos(θ_x)]

#rotate(θ_x,θ_y,θ_z) = rotate_z(θ_z) * rotate_y(θ_y) * rotate_x(θ_x)

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
    @constraint(model, m*(∂(v[1], t) - κ*v[1]v[2]               ) == tf * (throttleBrake(u[1], V) - F_aero(CdA,V) ) * cos(ψ)) # T
    @constraint(model, m*(κ*v[1]^2   + ∂(v[2], t)  - τ*v[1]*v[3]) == tf * (throttleBrake(u[1], V) - F_aero(CdA,V) ) * sin(ψ)) # N
    @constraint(model, m*(             τ*v[1]*v[2] + ∂(v[3], t) ) == tf * ( F_car_z  - F_aero(ClA, V) - m*g  ))


    # acceleration: these are not reeeallly used for the dynamics, but more to keep track of them for after
    # the fact during analysis. Might use them for weight transfer though, LLT*ay, for instance
    @constraint(model, [i = 1:3], ∂(v[i], t) == tf * a[i])
    # For now, much simplified velocity & yaw control 
    @constraint(model, ∂(ω, t) == tf * u[2]/1000)

    # update position state variables in TNB Ref Frame
        # only update x2,x3. x1 is always zero, as the TNB frame moves along the track
        # with the car, for now
    @constraint(model, [i = 2:3], ∂(x[i], t) == tf * v[i])
    
    @constraint(model, ∂(ψ, t) == tf * ω)

    # TNB Movement: update s by v1
    @constraint(model, ∂(s,t) == tf * v[1])

    # Car Velocity from velocity components
        # super weird, does not work with a norm, so using v1/cosψ
        # will need to update for a more complete rotation later...
    @constraint(model,V == getV(v[1],ψ))

    # Car forces
        # meant to simplify the TNB equations for readability, hopefully not adding too much solve time
        # in theory, could also apply control here, and coord system transformations
    #@constraint(model, F_car_x == throttleBrake(u[1], V)*(F_f[1]+ F_r[1]))
    #@constraint(model, F_car_y == )
    @constraint(model, F_car_z == cgDist*F_f[3] + (1-cgDist)*F_r[3])

# Enforce Track!
    # limit track width!
    @constraint(model, -2 <= x[2] <= 2)

    # car stays above track, but can lift off. 
    @constraint(model, x[3] >= 0)

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




# create named tuple to hold simulation data stuff
        # really tried to make this extensible, but had trouble with the datatypes
        # being GeneralVariableRef, not something more normal like a symbol or string
        # this will have to be updated as the model gets more complex,
        # no quick for loop or pretty Julia syntax 😟  
lapRunData = (
    t = value.(t) * value.(tf),
    s = value.(s),
    κ = value.(κ) * ones(length(value.(t))),
    τ = value.(τ) * ones(length(value.(t))),
    x = value.(x),
    v = value.(v),
    a = value.(a),
    V = value.(V),
    ψ = value.(ψ),
    ω = value.(ω),
    u = value.(u))

## Results Generation and Output
    println(solution_summary(optimizer_model(model)))

    println("Optimal final time is $(value.(tf)) seconds")


    # plots and plotts and plottts 📈

    #scatter(xw[1,:], xw[2,:], label = "Waypoints")
    #p3 = plot!(x_opt[1], x_opt[2], label = "Trajectory")
    #xlabel!("x_1")
    #ylabel!("x_2")

    plotTimeHistories(lapRunData)

    plotPosition(lapRunData)



        #(.5*zr25.air_density*zr25.Cd*zr25.frontal_area .* lapRunData.V .^2)
