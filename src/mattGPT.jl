using InfiniteOpt, Ipopt, Plots
include("vehicleParameters/carParametersModule.jl")
using .CarParametersModule
include("modelUtilities.jl")

# Car Spreadsheet and Parameters!
    #spreadsheetPath = "vehicleParameters\\zr25_data.xlsx"
    # now pulls from unified one stored in main matlab section of github, might move this somewhere else, who knows
spreadsheetPath = "C:\\Users\\benmo\\Documents\\GitHub\\Vehicle-Dynamics\\MATLAB\\vehicle_data\\zr25_data.xlsx"
car = CarParametersModule.create_car(spreadsheetPath)


# some parameters and initial conditions:
tfGuess = 7
trackLength = 75

    # initial conditions
    x0 = [0,0,0]
    v0 = [0,0,0]
    s0 = 0

    # piecewise definition of track:
    # index of track length segments
    sᵢ = [30, trackLength]
    # index of track curvature values
    #κᵢ = [0.005, 0.05]
    κᵢ = [0.00, 0.0]



## HERE STARTS THE INFINITE OPT STUFF!!!!!
model = InfiniteModel(optimizer_with_attributes(Ipopt.Optimizer, "print_level" => 0));

    # Car parameters, as finite parameters (see above)
    @finite_parameter(model, m == car.mass_total )
    @finite_parameter(model, l == car.wheelbase)
    @finite_parameter(model, cg_a == car.a)
    @finite_parameter(model, cg_b == car.b)
    @finite_parameter(model, tw_f == car.track_width_rear)
    @finite_parameter(model, tw_r == car.track_width_rear)
    @finite_parameter(model, cg_z == car.cg_height)
    @finite_parameter(model, cgDist == car.front_mass_distribution)
    @finite_parameter(model, CdA == car.Cd*car.frontal_area)       # 1.6 lmao
    @finite_parameter(model, ClA == car.Cl*car.frontal_area)
    @finite_parameter(model, r_wheel == car.tire_loaded_radius)
    #@finite_parameter(model, n_gearbox == car.gear_ratio)
    @finite_parameter(model, torque_drive_max == 21*car.gear_ratio)                                                    # Nm
    @finite_parameter(model, torque_brake_max_f == car.pad_friction_front*car.num_pistons_front*
                                car.disc_radius_front*car.piston_radius_front/car.mc_diameter_front*
                                car.brake_pedal_motion_ratio*car.balance_bar_ratio_front*car.max_pedal_force)        # Nm
    @finite_parameter(model, torque_brake_max_r == car.pad_friction_rear*car.num_pistons_rear*
                                car.disc_radius_rear*car.piston_radius_rear/car.mc_diameter_rear*
                                car.brake_pedal_motion_ratio*(1-car.balance_bar_ratio_front)*car.max_pedal_force)    # Nm
    @finite_parameter(model, μ == car.tire_mu*car.tire_mu_correction_factor*.5)

    @finite_parameter(model, g == car.g)
    @finite_parameter(model, ρ == car.air_density)

    @finite_parameter(model, longLT == car.long_load_transfer)
    @finite_parameter(model, lltd_f == car.lltd_front)
    @finite_parameter(model, lltd_r == car.lltd_rear)
    # this still worky.... blech
    @finite_parameter(model, I₁₁ == car.roll_polar_inertia)
    @finite_parameter(model, I₂₂ == car.pitch_polar_inertia)
    @finite_parameter(model, I₃₃ == car.yaw_polar_inertia)

    #m,l,cg_a,cg_b,tw_f,tw_r,cg_z,cgDist,CdA,ClA,r_wheel,torque_drive_max,torque_brake_max_f,torque_brake_max_r,μ,g,ρ,I₁₁,I₂₂,I₃₃ = inject_car_parameters!(model, car)
    # vector of wheel cp positions for yaw Generation
    # a = l-b
    rᵢ = [cg_a cg_a -cg_b -cg_b;tw_f/2 -tw_f/2 tw_r/2 -tw_r/2;-cg_z -cg_z -cg_z -cg_z]
    Iₚₒₒ = [I₁₁ 0 0;0 I₂₂ 0;0 0 I₃₃]
    

# Problem Variables
@infinite_parameter(model, t in [0, 1], num_supports = 70)

@variables(model, begin
    # track length
    s, Infinite(t)
    # state variables
    x[1:3], Infinite(t)
    v[1:3], Infinite(t)
    #a[1:3], Infinite(t)
    #V, Infinite(t)      # Car Velocity
    β, Infinite(t)      # Angle from car to velocity vector
    ψ, Infinite(t)      # Angle to TNB Frame
    ω, Infinite(t)      # Yaw rate
    
    #F_f[1:3], Infinite(t)
    #F_r[1:3], Infinite(t)

    Fₜ[1:3,1:4], Infinite(t)        # forces on tires.. x,y,z, then FL, FR, etc
    
    -15*π/180 ≤ α[1:4] ≤ 15*π/180, Infinite(t)
    δ[1:4], Infinite(t)
    #0 ≤ ωₜ[1:4], Infinite(t)

    #F_car[1:3], Infinite(t)
    #M_car[1:3], Infinite(t)

    # control variables
    -1 ≤ u[1:2] ≤ 1, Infinite(t)
end)

@variable(model, tf ≥ .8, start = tfGuess)
@expression(model, a[i=1:3], (1 / tf) * ∂(v[i], t))

# TNB Frame Stuff,

        T = [1;0;0]
        N = [0;1;0]
        B = [0;0;1]
        #κ = 0
        κ = InfiniteOpt.ifelse(s ≤ sᵢ[1], κᵢ[1], κᵢ[2])
        τ = 0
## 
# functions, test for control variables. 
    throttleMap(u,v) = u* (torque_drive_max/r_wheel)
    brakeMap(u) = u* (torque_brake_max_f + torque_brake_max_r)/2/r_wheel

    throttleBrakeMap(u,v) = InfiniteOpt.ifelse(u ≥ 0, throttleMap(u,v), brakeMap(u))

    # UPDATE TO MAX STEERING ANGLE STUFF
    steeringMap(u) = u* 15*π/180    # for now, 15° max at tire for 100° max at wheel (ish)

    F_aero(coeff,vel) = .5*ρ * coeff * vel^2
    
    #shittyTires(sa) = -20((sa*180/π)-10)^2 + 2000
    shittyTires(sa) = 2000*tanh(.2*sa*180/π)

    # defines skew symmetric cross product matrix thingyjawn for vector pp
    crossProductMatrix(pp) = [0 -pp[3] pp[2];pp[3] 0 -pp[1];-pp[2] pp[1] 0]
    

# Rotation matrix jawns- might be easier in axis angle formulation??
    rotate_z(θₖ) = [cos(θₖ) -sin(θₖ) 0; sin(θₖ) cos(θₖ) 0; 0 0 1]
    rotate_y(θⱼ) = [cos(θⱼ) 0 sin(θⱼ); 0 1 0; -sin(θⱼ) 0 cos(θⱼ)]
    rotate_x(θᵢ) = [1 0 0; 0 cos(θᵢ) -sin(θᵢ); 0 sin(θᵢ) cos(θᵢ)]
     
    rotate(θᵢ,θⱼ,θₖ) = rotate_z(θₖ) * rotate_y(θⱼ) * rotate_x(θᵢ)

    rotate_z_2d(θₖ) = [cos(θₖ) -sin(θₖ); sin(θₖ) cos(θₖ)]
    # Car Velocity from velocity components
        V = @expression(model, (sum(v[i]^2 for i=1:2))^.5)
    

## Objective Function! minimize final time
@objective(model,Min,tf)

# Initial Conditions
    @constraint(model, [i = 1:3], x[i](0) == x0[i])
    @constraint(model, [i = 1:3], v[i](0) == v0[i])
    @constraint(model, s(0) == s0)
    #@constraint(model, ψ(0) == 0)
    @constraint(model, β(0) == 0)
    @constraint(model, ω(0) == 0)

    # Make it go! s at final time is at the end of track (s(t=tf) = s_f)
    @constraint(model, s(1) == trackLength)

# Dynamic Constraints
    # Car Forces
        # AS CONSTRAINTS
        #@constraint(model, F_car .== rotate_z(ψ)*(Fₜ[:,1]+Fₜ[:,2]+Fₜ[:,3]+Fₜ[:,4] ))
       # @constraint(model, M_car .== crossProductMatrix(rᵢ[:,1])*Fₜ[:,1]+
        #                                crossProductMatrix(rᵢ[:,2])*Fₜ[:,2]+
         #                               crossProductMatrix(rᵢ[:,3])*Fₜ[:,3]+
          #                              crossProductMatrix(rᵢ[:,4])*Fₜ[:,4])
       
       
       # AS EXPRESSIONS
           # Forces and Yaw Moment on car
           F_car = @expression(model, rotate_z(ψ)*(Fₜ[:,1]+Fₜ[:,2]+Fₜ[:,3]+Fₜ[:,4] ))
           #M_car = @expression(model, crossProductMatrix(rᵢ[:,1])*Fₜ[:,1]+
           #                            crossProductMatrix(rᵢ[:,2])*Fₜ[:,2]+
           #                            crossProductMatrix(rᵢ[:,3])*Fₜ[:,3]+
           #                            crossProductMatrix(rᵢ[:,4])*Fₜ[:,4])
        

    # Dynamics in TNB Frame: hopefully this takes care of most of the Dynamics
            # also will need a basis for normal force, don't want to un-generalize it and say it's always
            # in the binormal direction. This would make it more sine-cosiney, should probably look more
            # into rotations
    #@constraint(model, m*(∂(v[1], t) - κ*v[1]v[2]               ) == tf * (F_car[1])) # T
    #@constraint(model, m*(κ*v[1]^2   + ∂(v[2], t)  - τ*v[1]*v[3]) == tf * (F_car[2])) # N
    #@constraint(model, m*(             τ*v[1]*v[2] + ∂(v[3], t) ) == tf * (F_car[3])) # B

    # For now, much simplified yaw control 
    #@constraint(model, Iₚₒₒ*[0;0;∂(ω, t)] .== tf * M_car)

    # Tire Forces        
        # Normal Forces/Weight Transfers: static + aero + lateral + longitudinal load transfer
        @constraint(model, Fₜ[3,1] == -cgDist*m*g/2     - 0*F_aero(ClA, V)/4 + a[1]*longLT/(2g) + 0*∂(v[2],t)*lltd_f/g)
        @constraint(model, Fₜ[3,2] == -cgDist*m*g/2     - 0*F_aero(ClA, V)/4 + a[1]*longLT/(2g) - 0*∂(v[2],t)*lltd_f/g)
        @constraint(model, Fₜ[3,3] == -(1-cgDist)*m*g/2 - 0*F_aero(ClA, V)/4 - a[1]*longLT/(2g) + 0*∂(v[2],t)*lltd_r/g)
        @constraint(model, Fₜ[3,4] == -(1-cgDist)*m*g/2 - 0*F_aero(ClA, V)/4 - a[1]*longLT/(2g) - 0*∂(v[2],t)*lltd_r/g)
        
        # Long/Lat forces:
        # apply control vectors to tires in tire coord. frames, translate to car frame
        @constraint(model, [i=1:4], Fₜ[1:2,i] .== rotate_z_2d(δ[i]) * [throttleBrakeMap(u[1], V), shittyTires(α[i])])

    
    # update position state variables in TNB Ref Frame
        # TNB Movement: update s by v1
        @constraint(model, ∂(s, t) == tf * v[1])
        # only update x2,x3. x1 is always zero, as the TNB frame moves along the track
        # with the car, for now
        @constraint(model, [i = 2:3], ∂(x[i], t) == tf * v[i])
    
        @constraint(model, ∂(β, t) == tf * ω)       # car to TNB??
        @constraint(model, ∂(ψ, t) == ω + v[1]*κ)   # global yaw rate

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
    #@constraint(model, u[1](0) >= 0)
    @constraint(model, u[2] == 0)
    #@constraint(model, u[2](0) == 0)
    @constraint(model, ω == 0)
    
    # these guys, eh, seems right. v3=0 for now, should be faster?
    #@constraint(model, v[1] >= 0)
    #@constraint(model, v[3] == 0)


# traction constraint, pls work!!!!
    #@constraint(model, [i = 1:2], u[i] <= maxTractionAvailable(F_car_z))
        # it worky! sorta, max traction not really well related to yaw rate..

    # acceleration: these are not reeeallly used for the dynamics, but more to keep track of them for after
    # the fact during analysis. Might use them for weight transfer though, LLT*ay, for instance
    #@constraint(model, [i = 1:3], ∂(v[i], t) == tf * a[i])
    #@expression(model, a[i=1:3], (1 / tf) * ∂(v[i], t))
        # But ay no work since v2 not in global frame!! needs inertial space somehow somewhere...? ugh
        # recover from curvature/velocity?

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
    κ = value.(κ),
    τ = value.(τ) * ones(length(value.(t))),
    x = value.(x),
    v = value.(v),
    a = value.(a),
    V = value.(V),
    β = value.(β),
    ψ = value.(ψ),
    ω = value.(ω),
    #ω_tnb = value.(κ) .* value.(v[1]),
    u = value.(u),
    FL = value.(Fₜ[:,1]),
    FR = value.(Fₜ[:,2]),
    RL = value.(Fₜ[:,3]),
    RR = value.(Fₜ[:,4]) )


## Results Generation and Output
    println(solution_summary(optimizer_model(model)))

    println("Optimal final time is $(value.(tf)) seconds")


    # plots and plotts and plottts 📈
    plotTimeHistories(lapRunData)

    plotPosition(lapRunData)

    plotTireForces(lapRunData)








   