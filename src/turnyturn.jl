using InfiniteOpt, Ipopt, Plots
include("vehicleParameters/carParametersModule.jl")
using .CarParametersModule
include("modelUtilities.jl")
include("tireJawn.jl")


# Car Spreadsheet and Parameters!
    # gotta update this to look in github, and not have path that only works for me 🤭
spreadsheetPath = "src\\vehicleParameters\\zr25_data.xlsx"
zr25 = CarParametersModule.create_car(spreadsheetPath)

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
    
    # piecewise definition of track:
        # index of track length segments
        sᵢ = [0,75]
        # index of track curvature values
        κᵢ = [0,.05]




## HERE STARTS THE INFINITE OPT STUFF!!!!!
model = InfiniteModel(optimizer_with_attributes(Ipopt.Optimizer, "print_level" => 0));

# Car parameters, as finite parameters (see above)
    @finite_parameter(model, m == zr25.mass_total)
    @finite_parameter(model, l == zr25.wheelbase)
    #@finite_parameter(model, a == zr25.a)
    @finite_parameter(model, b == zr25.b)
    @finite_parameter(model, tw_f == zr25.track_width_rear)
    @finite_parameter(model, tw_r == zr25.track_width_rear)
    @finite_parameter(model, cg_z == zr25.cg_height)
    @finite_parameter(model, cgDist == zr25.front_mass_distribution)
    @finite_parameter(model, CdA == (1.4+zr25.Cd*zr25.frontal_area))       # 1.6 lmao
    @finite_parameter(model, ClA == zr25.Cl*zr25.frontal_area)
    @finite_parameter(model, r_wheel == zr25.tire_loaded_radius)
    #@finite_parameter(model, n_gearbox == zr25.gear_ratio)
    @finite_parameter(model, torque_drive_max == 21*zr25.gear_ratio)                                                    # Nm
    @finite_parameter(model, torque_brake_max_f == zr25.pad_friction_front*zr25.num_pistons_front*
                                zr25.disc_radius_front*zr25.piston_radius_front/zr25.mc_diameter_front*
                                zr25.brake_pedal_motion_ratio*zr25.balance_bar_ratio_front*zr25.max_pedal_force)        # Nm
    @finite_parameter(model, torque_brake_max_r == zr25.pad_friction_rear*zr25.num_pistons_rear*
                                zr25.disc_radius_rear*zr25.piston_radius_rear/zr25.mc_diameter_rear*
                                zr25.brake_pedal_motion_ratio*(1-zr25.balance_bar_ratio_front)*zr25.max_pedal_force)    # Nm
    @finite_parameter(model, μ == zr25.tire_mu*zr25.tire_mu_correction_factor*.5)

    @finite_parameter(model, g == zr25.g)
    @finite_parameter(model, ρ == zr25.air_density)

    @finite_parameter(model, I₁₁ == 3)
    @finite_parameter(model, I₂₂ == 3)
    @finite_parameter(model, I₃₃ == 3)

    # vector of wheel cp positions for yaw Generation
    # a = l-b
    rᵢ = [(l-b) (l-b) -b -b;tw_f/2 -tw_f/2 tw_r/2 -tw_r/2;-cg_z -cg_z -cg_z -cg_z]
    Iₚₒₒ = [I₁₁ 0 0;0 I₂₂ 0;0 0 I₃₃]
    

# Problem Variables
@infinite_parameter(model, t in [0, tfGuess], num_supports = 200)


@variables(model, begin
    # track length
    s, Infinite(t)
    # state variables
    x[1:3], Infinite(t)
    v[1:3], Infinite(t)
    a[1:3], Infinite(t)
    #V, Infinite(t)      # Car Velocity
    β, Infinite(t)      # Angle from car to velocity vector
    ψ, Infinite(t)      # Angle to TNB Frame
    ω, Infinite(t)      # Yaw rate

    F_f[1:3], Infinite(t)
    F_r[1:3], Infinite(t)

    Fₜ[1:3,1:4], Infinite(t)
    
    -15*π/180 <= α[1:4] <= 15*π/180, Infinite(t)
    δ[1:4], Infinite(t)
    0 <= ωₜ[1:4], Infinite(t)

    F_car_z, Infinite(t)

    #F_car[1:3], Infinite(t)

    # control variables
    -1 <= u[1:2] <= 1, Infinite(t)
end)

@variable(model, tf >= 0, start = tfGuess)

# TNB Frame Stuff, attempt 2, straight line into curve
#=
    function get_κ(s₌)
        # function that takes the current value of s, and compares to s_vec
        # returns matching index position

        # non-general implementation for design day in 2 days,
        # need to be able to extend to arbitrary length vectors!!!
        return op_ifelse(op_less_than_or_equal_to(s₌, sᵢ[1]), κᵢ[1], κᵢ[2])
    end
    =#
        T = [1;0;0]
        N = [0;1;0]
        B = [0;0;1]
        κ = 0 #@expression(model, get_κ(s))
        τ = 0
## 
# functions, test for control variables. 
    throttleMap(u,v) = 4*(torque_drive_max/r_wheel)u #-.3v
    brakeMap(u) = (2*torque_brake_max_f + 2*torque_brake_max_r)u/r_wheel

    throttleBrakeMap(u,v) = InfiniteOpt.ifelse(u ≥ 0, throttleMap(u,v), brakeMap(u))

    steeringMap(u) = u*15 * π/180    # for now, 15° max at tire for 100° max at wheel (ish)



    F_aero(coeff,vel) = .5*ρ * coeff * vel^2
    #F_down = @expression(model, .5ρ*ClA* sum(v[i]^2 for i=1:2) )
    #F_drag = @expression(model, .5ρ*CdA* sum(v[i]^2 for i=1:2) )
    
    maxTractionAvailable(Fₙ) = (.25*(Fₙ)*μ) / (torque_drive_max/r_wheel)
    

    crossProductMatrix(pp) = [0 -pp[3] pp[2];pp[3] 0 -pp[1];-pp[2] pp[1] 0]
    

# Rotation matrix jawns- might be easier in axis angle formulation??
    rotate_z(θ_z) = [cos(θ_z) -sin(θ_z) 0; sin(θ_z) cos(θ_z) 0; 0 0 1]
    rotate_y(θ_y) = [cos(θ_y) 0 sin(θ_y); 0 1 0; -sin(θ_y) 0 cos(θ_y)]
    rotate_x(θ_x) = [1 0 0; 0 cos(θ_x) -sin(θ_x); 0 sin(θ_x) cos(θ_x)]
    rotate_z_2d(θ_z) = [cos(θ_z) -sin(θ_z); sin(θ_z) cos(θ_z)]

    rotate(θ_x,θ_y,θ_z) = rotate_z(θ_z) * rotate_y(θ_y) * rotate_x(θ_x)

    # Car Velocity from velocity components
        V = @expression(model, sqrt(sum(v[i]^2 for i=1:2)))
        @constraint(model, V ≥ 0)
    
    # Forces and Yaw Moment on car
    F_car = @expression(model, rotate_z(ψ)*(Fₜ[:,1]+Fₜ[:,2]+Fₜ[:,3]+Fₜ[:,4] ))
    M_car = @expression(model, crossProductMatrix(rᵢ[:,1])*Fₜ[:,1]+
                                crossProductMatrix(rᵢ[:,2])*Fₜ[:,2]+
                                crossProductMatrix(rᵢ[:,3])*Fₜ[:,3]+
                                crossProductMatrix(rᵢ[:,4])*Fₜ[:,4])
## Objective Function! minimize final time
@objective(model,Min,tf)

# Initial Conditions
    @constraint(model, [i = 1:3], x[i](0) == x0[i])
    @constraint(model, [i = 1:3], v[i](0) == v0[i])
    @constraint(model, s(0) == s0)
    @constraint(model, ψ(0) == 0)
    @constraint(model, β(0) == 0)
    @constraint(model, ω(0) == 0)
    @constraint(model, F_f[3](0) == cgDist*m*g)
    @constraint(model, F_r[3](0) == (1-cgDist)*m*g)
    
    # Make it go! s at final time is at the end of track (s(t=tf) = s_f)
    @constraint(model, s(tfGuess) == trackLength)

# Dynamic Constraints
    # Dynamics in TNB Frame: hopefully this takes care of most of the Dynamics
            # also will need a basis for normal force, don't want to un-generalize it and say it's always
            # in the binormal direction. This would make it more sine-cosiney, should probably look more
            # into rotations
    @constraint(model, m*(∂(v[1], t) - κ*v[1]v[2]               ) == tf * ((throttleBrakeMap(u[1],V) - F_aero(CdA,V))*cos(ψ))) # T
    @constraint(model, m*(κ*v[1]^2   + ∂(v[2], t)  - τ*v[1]*v[3]) == tf * ((throttleBrakeMap(u[1],V) - F_aero(CdA,V))* sin(ψ))) # N
    @constraint(model, m*(             τ*v[1]*v[2] + ∂(v[3], t) ) == tf * (F_car_z  - F_aero(ClsA,V) - m*g  ))


    # acceleration: these are not reeeallly used for the dynamics, but more to keep track of them for after
    # the fact during analysis. Might use them for weight transfer though, LLT*ay, for instance
    @constraint(model, [i = 1:3], ∂(v[i], t) == tf * a[i])
    
    # For now, much simplified velocity & yaw control 
    @constraint(model, Iₚₒₒ*[0;0;∂(ω, t)] .== tf * M_car)
    

    # update position state variables in TNB Ref Frame
        # only update x2,x3. x1 is always zero, as the TNB frame moves along the track
        # with the car, for now
        @constraint(model, [i = 2:3], ∂(x[i], t) == tf * v[i])
    
    #@constraint(model, ∂(ψ, t) == tf * ω)
    @constraint(model, ∂(β, t) == tf * ω)
    @constraint(model, ∂(ψ, t) == ω + v[1]*κ)

    # TNB Movement: update s by v1
    @constraint(model, ∂(s,t) == tf * v[1])



    # Car forces
        # transform tire forces in car frame to TNB Frame

        @constraint(model, F_car_z == F_f[3] + F_r[3])
        

        # for now, enforce static weight distribution. Eventually handle with lat, long x-fer
        @constraint(model, F_f[3] == cgDist*F_car_z)

    # Tire Forces
        # apply control vectors to tires in tire coord. frames, translate to car frame
    @constraint(model, [i=1:4], Fₜ[1:2,i] .== rotate_z_2d(δ[i]) * [.25*throttleBrakeMap(u[1], V),
                                                                shittyTires(α[i])])

    # Tire Model Stuff
        # steering
        @constraint(model, [i=1:2], δ[i] == steeringMap(u[2]))
        @constraint(model, [i=3:4], δ[i] == 0)        # no rear wheel deflection (for now)
        # Slip angles
        @constraint(model, [i=1:4], α[i] == β - δ[i])
        # Slip Ratios

    


# Enforce Track!
    # limit track width!
    @constraint(model, -2 <= x[2] <= 2)

    # car stays above track, but can lift off. 
    @constraint(model, x[3] >= 0)
    # need to enforce something similar for normal force vectors

# try to enforce some control vector continuity stuff
    # doesn't converge without this guy, which makes sense sorta
    @constraint(model, u[1](0) >= 0)
    #@constraint(model, u[2] == 0)
    @constraint(model, u[2](0) == 0)
    
    # these guys, eh, seems right. v3=0 for now, should be faster?
    @constraint(model, v[1] >= 0)
    @constraint(model, v[3] == 0)


# traction constraint, pls work!!!!
    @constraint(model, [i = 1:2], u[i] <= maxTractionAvailable(F_car_z))
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
    κ = value.(κ), #* ones(length(value.(t))),
    τ = value.(τ) * ones(length(value.(t))),
    x = value.(x),
    v = value.(v),
    a = value.(a),
    V = value.(V),
    β = value.(β),
    ψ = value.(ψ),
    ω = value.(ω),
    ω_tnb = value.(κ) .* value.(v[1]),
    u = value.(u))

## Results Generation and Output
    println(solution_summary(optimizer_model(model)))

    println("Optimal final time is $(value.(tf)) seconds")


    # plots and plotts and plottts 📈
    plotTimeHistories(lapRunData)

    plotPosition(lapRunData)