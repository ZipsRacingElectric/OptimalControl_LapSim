include("CarParametersModule.jl")
using .CarParametersModule


#=
Base.@kwdef struct InitCarParameters
    wheelbase::Float64
    track_width_front::Float64
    track_width_rear::Float64

    vehicle_mass::Float64
    driver_mass::Float64
    corner_mass_front::Float64
    corner_mass_rear::Float64
    front_mass_distribution::Float64
    cg_height::Float64
    yaw_polar_inertia::Float64
    roll_polar_inertia::Float64
    pitch_polar_inertia::Float64

    tire_loaded_radius::Float64
    tire_mu::Float64
    tire_mu_correction_factor::Float64
    tire_stiffness::Float64
    tire_width::Float64

    gear_ratio::Float64
    j_m::Float64
    J_s1::Float64
    J_1::Float64
    J_2::Float64
    J_s2::Float64
    J_w::Float64

    static_toe_front::Float64
    static_toe_rear::Float64
    static_camber_front::Float64
    static_camber_rear::Float64
    steering_ratio::Float64
    ackermann_percentage::Float64
    steering_arm_length::Float64
    steering_rack_length::Float64
    tie_rod_length_front::Float64
    steering_rack_to_axis_distance::Float64
    steering_pinion_radius::Float64
    roll_center_front::Float64
    roll_center_rear::Float64

    frontal_area::Float64
    Cd::Float64
    Cl::Float64
    center_of_pressure_distribution::Float64
    velocity_skidpad::Float64
    cla_at_skidpad::Float64
    cop_at_skidpad::Float64
    velocity_max::Float64
    cla_at_max_velocity::Float64
    cop_at_max_velocity::Float64

    damper_travel::Float64
    spring_rate_front::Float64
    spring_rate_rear::Float64
    bar_spring_rate_front::Float64
    bar_spring_rate_rear::Float64
    motion_ratio_front::Float64
    motion_ratio_rear::Float64
    bar_motion_ratio_front::Float64
    bar_motion_ratio_rear::Float64
    ride_frequency_front::Float64
    ride_frequency_rear::Float64

    toe_deflection_rear::Float64

    piston_radius_front::Float64
    piston_radius_rear::Float64
    num_pistons_front::Float64
    num_pistons_rear::Float64
    pad_friction_front::Float64
    pad_friction_rear::Float64
    max_pedal_force::Float64
    disc_radius_front::Float64
    disc_radius_rear::Float64
    pad_height_front::Float64
    pad_height_rear::Float64
    mc_diameter_front::Float64
    mc_diameter_rear::Float64
    balance_bar_ratio_front::Float64
    brake_pedal_motion_ratio::Float64

end =#

spreadsheetPath = "C:\\Users\\benmo\\Documents\\GitHub\\Vehicle-Dynamics\\MATLAB\\vehicle_data\\zr25_data.xlsx"

zr25 = create_car(spreadsheetPath)

