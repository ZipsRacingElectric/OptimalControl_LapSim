module CarParametersModule

using XLSX, DataFrames

export InitialCarParameters, carParameters, create_car, update_parameters, reset_parameters, get_used_parameters

# Immutable struct for storing the original parameters (no touchy!)
# Must be in Spreadsheet
Base.@kwdef struct InitialCarParameters
    #Dimensions
    wheelbase::Float64 = -1                                             #m
    track_width_front::Float64 = -1                                                   #m
    track_width_rear::Float64 = -1                                                    #m

    #Mass
    vehicle_mass::Float64 = -1                                                        #kg
    driver_mass::Float64 = -1                                                         #kg
    corner_mass_front::Float64 = -1                                                   #kg, unsprun corner mass. Include half of the control arm masses.
    corner_mass_rear::Float64 = -1                                                    #kg, unsprun corner mass. Include half of the control arm masses.
    front_mass_distribution::Float64 = -1                                             #percentage on front axle
    cg_height::Float64 = -1                                                           #m
    yaw_polar_inertia::Float64 = -1                                                   #kg * m^2, about the yaw (vertical) axis  of the C.G.
    roll_polar_inertia::Float64 = -1
    pitch_polar_inertia::Float64 = -1


    #Tires
    tire_loaded_radius::Float64 = -1                                                  #m
    gear_ratio::Float64 = -1                                                          #(# input rotations / # output rotations)
    tire_mu::Float64 = -1                                                             #the hot tire friction from TTC data                           
    tire_mu_correction_factor::Float64 = -1                                           #this is a correction factor to account for reduced tire-road friction compared to TTC data. Typically 2/3 but should be tuned if raining or cold outside
    tire_stiffness::Float64 = -1                                                      #N/m
    tire_width::Float64 = -1                                                          #m

    #Kinematics
    static_toe_front::Float64 = -1                                                    #degrees (per wheel), + is toe out
    static_toe_rear::Float64 = -1                                                     #degrees (per wheel), + is toe out
    static_camber_front::Float64 = -1                                                 #degrees, - is leaning torwards car
    static_camber_rear::Float64 = -1                                                  #degrees, - is leaning torwards car
    steering_ratio::Float64 = -1                                                      #ratio, steering wheel angle / ackerman steering angle (aka average of L and R angles)
    ackermann_percentage::Float64 = -1                                                #percentage
    steering_arm_length::Float64 = -1                                                 #m, perpendicular length between tire rod point and kingpin axis
    steering_rack_length::Float64 = -1                                                #m, eye to eye length of steering rack
    tie_rod_length_front::Float64 = -1                                               #m, perpendicular length between tire rod point and kingpin axis
    steering_rack_to_axis_distance::Float64 = -1                                      #m, distance between kingpin axis and steering rack, parallel to the longitudinal plane of the vehicle
    steering_pinion_radius::Float64 = -1                                              #m, radius of the steering rack pinion gear (reference for gear ratio calculation)
    roll_center_front::Float64 = -1                                                   #m, height of front roll center at static ride height
    roll_center_rear::Float64 = -1                                                    #m, height of rear roll center at static ride height

    # Gearbox inertias 
    Jm::Float64 = -1
    Js1::Float64 = -1
    J1::Float64 = -1
    J2::Float64 = -1
    Js2::Float64 = -1
    Jw::Float64 = -1


    #Aerodyamics
    frontal_area::Float64 = -1                                                        #m^2
    Cd::Float64 = -1                                                                  #unitless
    Cl::Float64 = -1                                                                  #unitless. Certain models may require it to be negative or positive based on implementation
    center_of_pressure_distribution::Float64 = -1                                     #ratio 0(at rear axle) to 1(at front axle)
    velocity_skidpad::Float64 = -1                                                    #velocity of skidpad for aero measurement
    cla_at_skidpad::Float64 = -1                                                      #unitless, + is downforce, ClA at skidpad
    cop_at_skidpad::Float64 = -1                                                      #ratio 0(at rear axle) to 1(at front axle), Cop at skidpad
    velocity_max::Float64 = -1                                                       #maximum velocity for aero measurement
    cla_at_max_velocity::Float64 = -1                                                #unitless, + is downforce,cla at max velocity
    cop_at_max_velocity::Float64 = -1                                                 #ratio 0(at rear axle) to 1(at front axle), CoP at max velocity

    #Springs and Dampers
    damper_travel::Float64 = -1                                                       #m, maximum travel of the damper
    spring_rate_front::Float64 = -1                                                  #N/m, spring rate at the damper
    spring_rate_rear::Float64 = -1                                                    #N/m, spring rate at the damper
    bar_spring_rate_front::Float64 = -1                                               #N/m, Spring rate of front roll bar
    bar_spring_rate_rear::Float64 = -1                                                #N/m, Spring rate of rear roll bar
    motion_ratio_front::Float64 = -1                                                  #unitless, Damper / wheel (assumes we use coil-overs)
    motion_ratio_rear::Float64 = -1                                                   #unitless, Damper / wheel (assumes we use coil-overs)
    bar_motion_ratio_front::Float64 = -1                                              #unitless, Roll bar / wheel (assumes we use coil-overs)
    bar_motion_ratio_rear::Float64 = -1                                              #unitless, Roll bar / wheel (assumes we use coil-overs)
    ride_frequency_front::Float64 = -1                                                #Hz, target front ride frequency (compare to calculated)
    ride_frequency_rear::Float64 = -1                                                 #Hz, target rear ride frequency (compare to calculated)

    #Compliance
    toe_deflection_rear::Float64 = -1                                                 #deg per 1kN, per wheel, linear toe deflection from Fy forces, from experimental testing

    #Brakes
    piston_radius_front::Float64 = -1                                                 #m
    piston_radius_rear::Float64 = -1                                                  #m
    num_pistons_front::Float64 = -1                                                   #unitless
    num_pistons_rear::Float64 = -1                                                    #unitless
    pad_friction_front::Float64 = -1                                                  #unitless, dynamic and static friction are assumed to be the same
    pad_friction_rear::Float64 = -1                                                   #unitless, dynamic and static friction are assumed to be the same
    max_pedal_force::Float64 = -1                                                     #N
    disc_radius_front::Float64 = -1                                                   #m
    disc_radius_rear::Float64 = -1                                                    #m
    pad_height_front::Float64 = -1                                                    #m
    pad_height_rear::Float64 = -1                                                     #m
    mc_diameter_front::Float64 = -1                                                   #m
    mc_diameter_rear::Float64 = -1                                                    #m
    balance_bar_ratio_front::Float64 = -1                                             #0 to 1
    brake_pedal_motion_ratio::Float64 = -1                                            #unitlessg


end

# Mutable struct for modifiable parameters
Base.@kwdef mutable struct carParameters
        #Dimensions
        wheelbase::Float64                                                           #m
        track_width_front::Float64                                                   #m
        track_width_rear::Float64                                                    #m

        #Mass
        vehicle_mass::Float64                                                        #kg
        driver_mass::Float64                                                         #kg
        corner_mass_front::Float64                                                   #kg, unsprun corner mass. Include half of the control arm masses.
        corner_mass_rear::Float64                                                    #kg, unsprun corner mass. Include half of the control arm masses.
        front_mass_distribution::Float64                                             #percentage on front axle
        cg_height::Float64                                                           #m
        yaw_polar_inertia::Float64                                                   #kg * m^2, about the yaw (vertical) axis  of the C.G.
        roll_polar_inertia::Float64
        pitch_polar_inertia::Float64


        #Tires
        tire_loaded_radius::Float64                                                  #m
        gear_ratio::Float64                                                          #(# input rotations / # output rotations)
        tire_mu::Float64                                                             #the hot tire friction from TTC data                           
        tire_mu_correction_factor::Float64                                           #this is a correction factor to account for reduced tire-road friction compared to TTC data. Typically 2/3 but should be tuned if raining or cold outside
        tire_stiffness::Float64                                                      #N/m
        tire_width::Float64                                                          #m
        
        #Kinematics
        static_toe_front::Float64                                                    #degrees (per wheel), + is toe out
        static_toe_rear::Float64                                                     #degrees (per wheel), + is toe out
        static_camber_front::Float64                                                 #degrees, - is leaning torwards car
        static_camber_rear::Float64                                                  #degrees, - is leaning torwards car
        steering_ratio::Float64                                                      #ratio, steering wheel angle / ackerman steering angle (aka average of L and R angles)
        ackermann_percentage::Float64                                                #percentage
        steering_arm_length::Float64                                                 #m, perpendicular length between tire rod point and kingpin axis
        steering_rack_length::Float64                                                #m, eye to eye length of steering rack
        tie_rod_length_front::Float64                                               #m, perpendicular length between tire rod point and kingpin axis
        steering_rack_to_axis_distance::Float64                                      #m, distance between kingpin axis and steering rack, parallel to the longitudinal plane of the vehicle
        steering_pinion_radius::Float64                                              #m, radius of the steering rack pinion gear (reference for gear ratio calculation)
        roll_center_front::Float64                                                   #m, height of front roll center at static ride height
        roll_center_rear::Float64                                                    #m, height of rear roll center at static ride height
        
        # Gearbox inertias 
        Jm::Float64
        Js1::Float64
        J1::Float64
        J2::Float64
        Js2::Float64
        Jw::Float64


        #Aerodyamics
        frontal_area::Float64                                                        #m^2
        Cd::Float64                                                                  #unitless
        Cl::Float64                                                                  #unitless. Certain models may require it to be negative or positive based on implementation
        center_of_pressure_distribution::Float64                                     #ratio 0(at rear axle) to 1(at front axle)
        velocity_skidpad::Float64                                                    #velocity of skidpad for aero measurement
        cla_at_skidpad::Float64                                                      #unitless, + is downforce, ClA at skidpad
        cop_at_skidpad::Float64                                                      #ratio 0(at rear axle) to 1(at front axle), Cop at skidpad
        velocity_max::Float64                                                       #maximum velocity for aero measurement
        cla_at_max_velocity::Float64                                                #unitless, + is downforce,cla at max velocity
        cop_at_max_velocity::Float64                                                 #ratio 0(at rear axle) to 1(at front axle), CoP at max velocity

        #Springs and Dampers
        damper_travel::Float64                                                       #m, maximum travel of the damper
        spring_rate_front::Float64                                                  #N/m, spring rate at the damper
        spring_rate_rear::Float64                                                    #N/m, spring rate at the damper
        bar_spring_rate_front::Float64                                               #N/m, Spring rate of front roll bar
        bar_spring_rate_rear::Float64                                                #N/m, Spring rate of rear roll bar
        motion_ratio_front::Float64                                                  #unitless, Damper / wheel (assumes we use coil-overs)
        motion_ratio_rear::Float64                                                   #unitless, Damper / wheel (assumes we use coil-overs)
        bar_motion_ratio_front::Float64                                              #unitless, Roll bar / wheel (assumes we use coil-overs)
        bar_motion_ratio_rear::Float64                                              #unitless, Roll bar / wheel (assumes we use coil-overs)
        ride_frequency_front::Float64                                                #Hz, target front ride frequency (compare to calculated)
        ride_frequency_rear::Float64                                                 #Hz, target rear ride frequency (compare to calculated)

        #Compliance
        toe_deflection_rear::Float64                                                 #deg per 1kN, per wheel, linear toe deflection from Fy forces, from experimental testing

        #Brakes
        piston_radius_front::Float64                                                 #m
        piston_radius_rear::Float64                                                  #m
        num_pistons_front::Float64                                                   #unitless
        num_pistons_rear::Float64                                                    #unitless
        pad_friction_front::Float64                                                  #unitless, dynamic and static friction are assumed to be the same
        pad_friction_rear::Float64                                                   #unitless, dynamic and static friction are assumed to be the same
        max_pedal_force::Float64                                                     #N
        disc_radius_front::Float64                                                   #m
        disc_radius_rear::Float64                                                    #m
        pad_height_front::Float64                                                    #m
        pad_height_rear::Float64                                                     #m
        mc_diameter_front::Float64                                                   #m
        mc_diameter_rear::Float64                                                    #m
        balance_bar_ratio_front::Float64                                             #0 to 1
        brake_pedal_motion_ratio::Float64                                            #unitlessg

    ## Calculated parameters - call update function to calculate
            ## Julia requires that all parameters be initialized when struct is created,
            # so calculated parameters will use default values of -1, which will be updated
            # when the update function is called
    # Dimensions
    a::Float64 = -1                                                                   # m, distance from front axle to CG
    b::Float64 = -1                                                                  # m, distance from rear axle to CG
    b_sprung::Float64 = -1                                                            # m, distance from rear axle to sprung CG
    a_sprung::Float64 = -1                                                            # m, distance from front axle to sprung CG
    cg_sprung::Float64 = -1                                                           # m, height of sprung CG above ground
    average_track_width::Float64 = -1                                                 # m, average track width (front and rear)
    cg_track_ratio::Float64 = -1                                                      # unitless, ratio of CG height to mean track wid

    # Mass
    mass_total::Float64 = -1                                                          # kg, total vehicle mass
    sprung_mass_total::Float64 = -1                                                   # kg, total sprung mass
    sprung_mass_front::Float64 = -1                                                   # kg, front axle sprung mass
    sprung_mass_rear::Float64 = -1                                                    # kg, rear axle sprung mass
    unsprung_mass_total::Float64 = -1                                                 # kg, total unsprung mass
    unsprung_mass_front::Float64 = -1                                                 # kg, front axle unsprung mass
    unsprung_mass_rear::Float64 = -1                                                  # kg, rear axle unsprung mass
    average_corner_mass::Float64 = -1                                                 # kg, average mass at each corner
    
    front_mass::Float64 = -1                                                        # kg, total front axle weight
    rear_mass::Float64 = -1                                                         # kg, total rear axle weight
    front_corner_mass::Float64 = -1                                                 # kg, average front corner weight
    rear_corner_mass::Float64 = -1                                                  # kg, average rear corner weight

    # Springs and Dampers
    wheel_rate_front::Float64 = -1                                                    # N/m, effective front wheel rate
    wheel_rate_rear::Float64 = -1                                                     # N/m, effective rear wheel rate
    wheel_rate_from_bar_front::Float64 = -1                                           # N/m, contribution of front anti-roll bar to wheel rate
    wheel_rate_from_bar_rear::Float64 = -1                                            # N/m, contribution of rear anti-roll bar to wheel rate
    spring_rate_total_front::Float64 = -1                                             # N/m, combined front spring rate
    spring_rate_total_rear::Float64 = -1                                              # N/m, combined rear spring rate
    roll_wheel_rate_front::Float64 = -1                                               # N/m, total front roll wheel rate
    roll_wheel_rate_rear::Float64 = -1                                                # N/m, total rear roll wheel rate
    total_roll_rate_front::Float64 = -1                                               # N/m, total front roll rate (including tires)
    total_roll_rate_rear::Float64 = -1                                                # N/m, total rear roll rate (including tires)

    # Aerodynamics
    downforce_at_skidpad_front::Float64 = -1                                          # N, aerodynamic downforce on front axle at skidpad velocity
    downforce_at_skidpad_rear::Float64 = -1                                           # N, aerodynamic downforce on rear axle at skidpad velocity
    downforce_at_max_velocity_front::Float64 = -1                                     # N, aerodynamic downforce on front axle at max velocity
    downforce_at_max_velocity_rear::Float64 = -1                                      # N, aerodynamic downforce on rear axle at max velocity

    # Lateral Load Transfer Distribution
    height_nra_to_sm_center::Float64 = -1                                             # m, height of neutral roll axis relative to sprung mass CG
    kf::Float64 = -1                                                                  # Nm/deg, front roll stiffness
    kr::Float64 = -1                                                                  # Nm/deg, rear roll stiffness
    kf_prime::Float64 = -1                                                            # Nm/deg, adjusted front roll stiffness
    kr_prime::Float64 = -1                                                            # Nm/deg, adjusted rear roll stiffness
    lltd_front::Float64 = -1                                                          # percent, lateral load transfer distribution on front axle
    lltd_rear::Float64 = -1                                                           # percent, lateral load transfer distribution on rear axle
    tlltd::Float64 = -1                                                               # percent, total lateral load transfer distribution

    # Longitudinal Load Transfer
    long_load_transfer::Float64 = -1                                                  # N, longitudinal load transfer force

    ## Constants
    # Environment
    air_temp::Float64 = 20;                                                      # degrees C
    air_density::Float64 = 1.225;                                                # kg / m^3, at 20 degrees C
    g::Float64 = 9.81;                                                           # m/s^2, acceleration of gravity


    original::InitialCarParameters  # Stores immutable reference to original values
end

# Function to create a car object from a CSV file
function create_car(spreadsheetPath::String)
    parametersSpreadsheet = XLSX.readxlsx(spreadsheetPath)
    parametersSheet = parametersSpreadsheet["parameters"]

    # read excel data into a data frame thingy
    df = XLSX.gettable(parametersSheet,"A:B",first_row=3,
                        stop_in_empty_row=false,
                        column_labels = ["parameter","value"],
                        infer_eltypes = false)|> DataFrame

    # convert types in df to match struct
    df.parameter = Symbol.(string.(df.parameter))
    df.value = Float64.(df.value)

    # filter out anything that might have been added, and not yet in this code
    usedParameters = get_used_parameters()
    df = filter(row -> row.parameter in usedParameters, df)

    # convert to dict then named tuple because ????      weird...
    parameterTuple = Dict(df.parameter .=> df.value)
    parameterTuple = (; parameterTuple...)

    # create initial parameter struct with ^ that stuff
    original = InitialCarParameters(; parameterTuple...)  # Spread NamedTuple values
    println("Initial Parameters Read from Spreadsheet")
    # Creates car struct, calculated parameters are unnassigned and have default value of -1 
    car = carParameters(; parameterTuple...,  original=original)
    
    update_parameters(car)

    return car
end


# Function to update derived parameters
function update_parameters(car::carParameters)
    # Calculates all parameters derived from basic parameters
    # Mass
    car.mass_total = car.vehicle_mass + car.driver_mass;
    car.unsprung_mass_front = 2 * car.corner_mass_front;
    car.unsprung_mass_rear = 2 * car.corner_mass_rear;
    car.unsprung_mass_total = car.unsprung_mass_front + car.unsprung_mass_rear;
    car.sprung_mass_total = car.mass_total - car.unsprung_mass_total;
    car.sprung_mass_front = (car.mass_total * car.front_mass_distribution) - car.unsprung_mass_front;
    car.sprung_mass_rear = car.sprung_mass_total - car.sprung_mass_front;
    car.average_corner_mass = car.unsprung_mass_total / 4;
    car.front_mass = car.mass_total * car.front_mass_distribution;
    car.rear_mass = car.mass_total * (1 - car.front_mass_distribution);
    car.front_corner_mass = car.front_mass / 2;
    car.rear_corner_mass = car.rear_mass / 2;
    
    # Dimensions
    car.a = car.wheelbase * (1 - car.front_mass_distribution);
    car.b = car.wheelbase * (car.front_mass_distribution);          
    car.b_sprung = (car.mass_total * car.b - car.unsprung_mass_front * car.wheelbase) / car.sprung_mass_total;
    car.a_sprung = car.wheelbase - car.b_sprung;
    car.cg_sprung = ((car.mass_total * car.cg_height) - (car.unsprung_mass_front * car.tire_loaded_radius) - (car.unsprung_mass_rear * car.tire_loaded_radius)) / car.sprung_mass_total;
    car.average_track_width = (car.track_width_front + car.track_width_rear) / 2;
    car.cg_track_ratio = car.cg_height / car.average_track_width;
    
    # Spring and Dampers
    car.wheel_rate_front = car.spring_rate_front * car.motion_ratio_front^2;
    car.wheel_rate_rear = car.spring_rate_rear * car.motion_ratio_rear^2;
    car.wheel_rate_from_bar_front = car.bar_spring_rate_front * car.bar_motion_ratio_front^2;
    car.wheel_rate_from_bar_rear = car.bar_spring_rate_rear * car.bar_motion_ratio_rear^2;
    car.spring_rate_total_front = (car.wheel_rate_front * car.tire_stiffness) / (car.wheel_rate_front + car.tire_stiffness);
    car.spring_rate_total_rear = (car.wheel_rate_rear * car.tire_stiffness) / (car.wheel_rate_rear + car.tire_stiffness);
    car.roll_wheel_rate_front = car.wheel_rate_front + car.wheel_rate_from_bar_front;
    car.roll_wheel_rate_rear = car.wheel_rate_rear + car.wheel_rate_from_bar_rear;
    car.total_roll_rate_front = (car.roll_wheel_rate_front * car.tire_stiffness) / (car.roll_wheel_rate_front + car.tire_stiffness);
    car.total_roll_rate_rear = (car.roll_wheel_rate_rear * car.tire_stiffness) / (car.roll_wheel_rate_rear + car.tire_stiffness);

    # Aerodynamics
    car.downforce_at_skidpad_front = (car.cla_at_skidpad * car.air_density * 0.5 * car.velocity_skidpad^2) * car.center_of_pressure_distribution / car.g;
    car.downforce_at_skidpad_rear = (car.cla_at_skidpad * car.air_density * 0.5 * car.velocity_skidpad^2) * (1 - car.center_of_pressure_distribution) / car.g;
    car.downforce_at_max_velocity_front = (car.cla_at_max_velocity * car.air_density * 0.5 * car.velocity_max^2) * car.cop_at_max_velocity / car.g;
    car.downforce_at_max_velocity_rear = (car.cla_at_max_velocity * car.air_density * 0.5 * car.velocity_max^2) * (1 - car.cop_at_max_velocity) / car.g;

    # Lateral Load Transfer Calculations
    car.height_nra_to_sm_center = abs((car.roll_center_rear - car.roll_center_front) * (car.a - car.a_sprung) - (-car.b - car.a) * car.cg_sprung + (-car.b) * car.roll_center_front - car.roll_center_rear * car.a) / sqrt((car.roll_center_front - car.roll_center_rear)^2 + (-car.b - car.a)^2);
    car.kf = 0.5 * car.total_roll_rate_front * car.track_width_front^2;
    car.kr = 0.5 * car.total_roll_rate_rear * car.track_width_rear^2;
    car.kf_prime = car.kf - (car.wheelbase - car.a_sprung) * car.sprung_mass_total * car.g * car.height_nra_to_sm_center / car.wheelbase;
    car.kr_prime = car.kr - car.a_sprung * car.sprung_mass_total * car.height_nra_to_sm_center / car.wheelbase;
    car.lltd_front = calculate_lateral_load_transfer_front(car);
    car.lltd_rear = calculate_lateral_load_transfer_rear(car)
    car.tlltd = car.lltd_front / (car.lltd_front + car.lltd_rear);

    # Longitudinal Load Transfer Calculations
    car.long_load_transfer = car.cg_height / car.wheelbase * car.mass_total * car.g;

    println("Derived Parameters Updated")
end

# Function to reset parameters to the original values
function reset_parameters(car::carParameters)

    for name in fieldnames(typeof(car.original))
        setfield!(car, name, getfield(car.original, name))
    end
#=
    car.wheelbase=car.original.wheelbase
    car.track_width_front=car.original.track_width_front
    car.track_width_rear=car.original.track_width_rear

    car.vehicle_mass=car.original.vehicle_mass
    car.driver_mass=car.original.driver_mass
    car.corner_mass_front=car.original.corner_mass_front
    car.corner_mass_rear=car.original.corner_mass_rear
    car.front_mass_distribution=car.original.front_mass_distribution
    car.cg_height=car.original.cg_height
    car.yaw_polar_inertia=car.original.yaw_polar_inertia
    car.roll_polar_inertia=car.original.roll_polar_inertia
    car.pitch_polar_inertia=car.original.pitch_polar_inertia

    car.tire_loaded_radius=car.original.tire_loaded_radius
    car.tire_mu=car.original.tire_mu
    car.tire_mu_correction_factor=car.original.tire_mu_correction_factor
    car.tire_stiffness=car.original.tire_stiffness
    car.tire_width=car.original.tire_width

    car.gear_ratio=car.original.gear_ratio
    car.Jm=car.original.Jm
    car.Js1=car.original.Js1
    car.J1=car.original.J1
    car.J2=car.original.J2
    car.Js2=car.original.Js2
    car.Jw=car.original.Jw

    car.static_toe_front=car.original.static_toe_front
    car.static_toe_rear=car.original.static_toe_rear
    car.static_camber_front=car.original.static_camber_front
    car.static_camber_rear=car.original.static_camber_rear
    car.steering_ratio=car.original.steering_ratio
    car.ackermann_percentage=car.original.ackermann_percentage
    car.steering_arm_length=car.original.steering_arm_length
    car.steering_rack_length=car.original.steering_rack_length
    car.tie_rod_length_front=car.original.tie_rod_length_front
    car.steering_rack_to_axis_distance=car.original.steering_rack_to_axis_distance
    car.steering_pinion_radius=car.original.steering_pinion_radius
    car.roll_center_front=car.original.roll_center_front
    car.roll_center_rear=car.original.roll_center_rear

    car.frontal_area=car.original.frontal_area
    car.Cd=car.original.Cd
    car.Cl=car.original.Cl
    car.center_of_pressure_distribution=car.original.center_of_pressure_distribution
    car.velocity_skidpad=car.original.velocity_skidpad
    car.cla_at_skidpad=car.original.cla_at_skidpad
    car.cop_at_skidpad=car.original.cop_at_skidpad
    car.velocity_max=car.original.velocity_max
    car.cla_at_max_velocity=car.original.cla_at_max_velocity
    car.cop_at_max_velocity=car.original.cop_at_max_velocity

    car.damper_travel=car.original.damper_travel
    car.spring_rate_front=car.original.spring_rate_front
    car.spring_rate_rear=car.original.spring_rate_rear
    car.bar_spring_rate_front=car.original.bar_spring_rate_front
    car.bar_spring_rate_rear=car.original.bar_spring_rate_rear
    car.motion_ratio_front=car.original.motion_ratio_front
    car.motion_ratio_rear=car.original.motion_ratio_rear
    car.bar_motion_ratio_front=car.original.bar_motion_ratio_front
    car.bar_motion_ratio_rear=car.original.bar_motion_ratio_rear
    car.ride_frequency_front=car.original.ride_frequency_front
    car.ride_frequency_rear=car.original.ride_frequency_rear

    car.toe_deflection_rear=car.original.toe_deflection_rear

    car.piston_radius_front=car.original.piston_radius_front
    car.piston_radius_rear=car.original.piston_radius_rear
    car.num_pistons_front=car.original.num_pistons_front
    car.num_pistons_rear=car.original.num_pistons_rear
    car.pad_friction_front=car.original.pad_friction_front
    car.pad_friction_rear=car.original.pad_friction_rear
    car.max_pedal_force=car.original.max_pedal_force
    car.disc_radius_front=car.original.disc_radius_front
    car.disc_radius_rear=car.original.disc_radius_rear
    car.pad_height_front=car.original.pad_height_front
    car.pad_height_rear=car.original.pad_height_rear
    car.mc_diameter_front=car.original.mc_diameter_front
    car.mc_diameter_rear=car.original.mc_diameter_rear
    car.balance_bar_ratio_front=car.original.balance_bar_ratio_front
    car.brake_pedal_motion_ratio=car.original.brake_pedal_motion_ratio
    =#

    println("Parameters Reset to original values")
    update_parameters(car)    

end




function calculate_lateral_load_transfer_front(car::carParameters)
    # Calculate the lateral load transfer distribution for the front axle
    #front_roll_center = car.roll_center_front;
    #rear_roll_center = car.roll_center_rear;

    # this is an assumption!!
    height_sprung_mass_cg = car.cg_sprung;

    lltd_front = (car.sprung_mass_total * car.g / car.track_width_front) * 
        ((car.height_nra_to_sm_center * car.kf_prime) / (car.kf + car.kr - car.sprung_mass_total * car.g * car.height_nra_to_sm_center) +
        (car.wheelbase - car.a_sprung) / car.wheelbase * car.roll_center_front) +
        car.unsprung_mass_front * car.g / car.track_width_front * height_sprung_mass_cg;

    return lltd_front
end

function calculate_lateral_load_transfer_rear(car::carParameters)
        # this is an assumption!!
        height_sprung_mass_cg = car.cg_sprung;

    lltd_rear = (car.sprung_mass_total * car.g / car.track_width_rear) * 
        ((car.height_nra_to_sm_center * car.kr_prime) / (car.kf + car.kr - car.sprung_mass_total * car.g * car.height_nra_to_sm_center) +
        (car.a_sprung) / car.wheelbase * car.roll_center_rear) + 
        car.unsprung_mass_rear * car.g / car.track_width_rear * height_sprung_mass_cg;

        return lltd_rear
end

function calculate_dragforce(car::carParameters, velocity::Float64)
    # Calculate the down force given a velocity
    return 0.5 * car.air_density * velocity^2 * car.Cd * car.frontal_area;
end

function calculate_downforce(car::carParameters, velocity::Float64)
    # Calculate the aerodynamic downforce given a velocity
    return 0.5 * car.air_density * velocity^2 * car.Cl * car.frontal_area;
end

function get_used_parameters()
    # this jawn returns a vector of symbols from the initial car parameters struct
    # used to filter only the working, used parameters in this combined
    # from the spreadsheet. 

    dummy = InitialCarParameters()  # Create a default instance
    return collect(propertynames(dummy))

end

end  # End of module