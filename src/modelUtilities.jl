using InfiniteOpt, Plots, LinearAlgebra


function inject_car_parameters!(model::InfiniteModel, car::carParameters)
    # this function hides the ugly mess of adding all the relevant parameters from the carParametersModule.jl
    # into the infiniteOpt model. This might be made a separate module at some point, or baked into 
    # carParameters directly or something... for now, hide mess. I like this structure though, hints at 
    # future modularity, see https://chatgpt.com/share/68242abc-0564-800d-aa28-8ca73825aad4
    # (it was all me not chatGPT!!!!!)

    # unfortunately, most of the parameters I want to pull from this, will be better to include
    # as finite parameters in the infiniteOpt model, so cant use directly, will have dumb stuff
    # like @parameter(model, m, zr25.mass) or something. This is to allow more human readable
    # equations in the model, rather than scalars, and one can supposedly perform some
    # sensitivity analysis, according to chatGPT (lol)

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

    # no want worky hmmmmmm....
    #@finite_parameter(model, I₁₁ == car.roll_polar_inertia)
    #@finite_parameter(model, I₂₂ == car.pitch_polar_inertia)
    #@finite_parameter(model, I₃₃ == car.yaw_polar_inertia)

    # this still worky.... blech
    @finite_parameter(model, I₁₁ == 3 )#car.roll_polar_inertia)
    @finite_parameter(model, I₂₂ == 3 )#car.pitch_polar_inertia)
    @finite_parameter(model, I₃₃ == 17 )# car.yaw_polar_inertia)
    return (; m, l, cg_a, cg_b, tw_f, tw_r, cg_z,cgDist, CdA, ClA, r_wheel, torque_drive_max, torque_brake_max_f, torque_brake_max_r, μ, g, ρ, I₁₁, I₂₂, I₃₃)
end


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
    n = 3
    v_mat = hcat(runData.v[1:2]...,runData.V)  # now columns are inputs
    
    p1 = Plots.plot(runData.t, runData.u; ylabel = "Control Input",
                    label = ["Accelerator/Brake Input" "Steering Wheel Input"],
                    legend = :bottomright,
                    guidefont = font(7,"Computer Modern"),
                    title = "Control Inputs and State Vector Traces",
                    titlefont = font(12,"Computer Modern"))
    p2 = Plots.plot(runData.t, runData.v[1:2]; ylabel = "Velocity (m\\/s)",
                    label = ["v₁" "v₂" "V" ],
                    guidefont = font(7,"Computer Modern"))
    
    p3 = Plots.plot(runData.t, runData.V; ylabel = "Speed")
    p4 = Plots.plot(runData.t, runData.a[1:2]; ylabel = "Acceleration (m\\/s^{2})", xlabel = "Time (s)",
                    label = ["a₁" "a₂" "a₃" "V" ],
                    legend = :bottomright,
                    guidefont = font(7,"Computer Modern"))
    p5 = Plots.plot(runData.t, runData.ψ; ylabel = "Angle to TNB, ψ")
        
    plotJawn = plot(p1,p2,p4, layout=(n,1))
                #guidefont = font(9,"Computer Modern")))
    display(plotJawn)

    return plotJawn 
end

function plotPosition(runData)
    # Plots plan view of car along path

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
    x_left_border = zeros(length(runData.s), 3)
    x_right_border = zeros(length(runData.s), 3)
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
        x_left_border[i, :] = x_track[i, :] + 2*N[i, :]
        x_right_border[i, :] = x_track[i, :] - 2*N[i, :]
        x_car[i, :] = x_track[i, :]
    end
    x_car = x_car + lapRunData.x[2] .* N 
    
    gr()

    p1 = Plots.plot(x_track[:,1], x_track[:,2], label = "Centerline",
                    xlabel = "X (m)",
                    ylabel = "Y (m)",
                    title = "Vehicle Path on Track",
                    guidefont = font(9,"Computer Modern"),
                    titlefont = font(12,"Computer Modern"))
    plot!(p1,x_car[:,1],x_car[:,2],label="Racecar's Path",lw = 3)
    plot!(p1,x_left_border[:,1],x_left_border[:,2], label = "limit₊", lc=:grey,lw = 1)
    plot!(p1,x_right_border[:,1],x_right_border[:,2], label = "limit₋", lc=:grey, lw = 1)

    display(plot(p1,layout=(1,1)))

    return p1

end

function plotTireForces(runData)
    n = 4
    #v_mat = hcat(runData.v...,runData.V)  # now columns are inputs
    
    p1 = plot(runData.t, runData.FL; ylabel = "Force (N)",
              label = ["FL_X" "FL_Y" "FL_Z"], legend = :bottomright,
              title = "Front Left")

    p2 = plot(runData.t, runData.FR; ylabel = "Force (N)",
              label = ["FR_X" "FR_Y" "FR_Z"], legend = :bottomright,
              title = "Front Right")

    p3 = plot(runData.t, runData.RL; ylabel = "Force (N)",
              label = ["RL_X" "RL_Y" "RL_Z"], legend = :bottomright,
              title = "Rear Left")

    p4 = plot(runData.t, runData.RR; ylabel = "Force (N)", xlabel = "Time (s)",
              label = ["RR_X" "RR_Y" "RR_Z"], legend = :bottomright,
              title = "Rear Right")

    
    plotJawn = plot(p1,p2,p3,p4; layout=(2,2), size = (1500, 1200))         
    display(plotJawn)

    return plotJawn 
end