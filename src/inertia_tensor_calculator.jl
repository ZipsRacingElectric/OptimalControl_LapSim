### Inertia Tensor With 4 Wheels, Accumulator, body, & Driver
### Disclaimer: Many numbers here are too specific and / or are rough estimates

## Dimensions

a = 0.7946 # m
b = 0.7344 # m
htf = 1230/1000/2 # m (half-track front)
htr = 1210/1000/2 # m (half-track rear)
r = 0.259 # m (tire radius)
acc_l_x = 0.25 # m (accumulator length along x)
acc_w_y = 0.5 # m (accumulator width along y)
acc_h_z = 0.25 # m (accumulator height along z)
body_l_x = 2.36 # m (body length along x)
body_w_y = 0.56 # m (body width along y)
body_h_z = 0.46 # m (body height along z)
body_t = 0.005 # m (body thickness)
d_l_x = 1.78 # m (driver's height)
d_w_y = 0.36 # m (driver's width (hip-to-hip))
d_h_z = 0.23 # m (driver's thickness (lower back to pants button))

## Masses & Positions
# Each vector, in accending element, goes tire, tire, tire, tire, accululator, body, driver's sacrum (look it up)

masses = [19.1, 19.1, 19.1, 19.1, 60, body_mass, 77] # kg
pos_x = [a, a, -b, -b, -0.465, body_pos_x, 0] # m
pos_y = [htf, -htf, htr, -htr, 0, body_pos_y, 0] # m
pos_z = [r, r, r, r, 0.238, body_pos_z, 0.285] # m

## Center of Mass & Inertia Tensor Calculations

pos_x_ave = sum(pos_x) / length(pos_x)
pos_y_ave = sum(pos_y) / length(pos_y)
pos_z_ave = sum(pos_z) / length(pos_z)

pos_x2 = pos_x .- pos_x_ave
pos_y2 = pos_y .- pos_y_ave
pos_z2 = pos_z .- pos_z_ave

tire_inertia_tensor = masses[1]*[0.25 0 0; 0 0.5 0; 0 0 0.25]*r^2 # Tires are short cylinders
accumulator_inertia_tensor = masses[5]*[(acc_w_y^2+acc_h_z^2) 0 0; 0 (acc_l_x^2+acc_h_z^2) 0; 0 0 (acc_l_x^2+acc_w_y^2)]/12 # Big box
body_inertia_tensor = masses[6]*[(body_w_y^2+body_h_z^2-(body_w_y-body_t)^2-(body_h_z-body_t)^2) 0 0; 0 (body_l_x^2+body_h_z^2-(body_l_x-body_t)^2-(body_h_z-body_t)^2) 0; 0 0 (body_l_x^2+body_w_y^2-(body_l_x-body_t)^2-(body_w_y-body_t)^2)]/12 # The car body is one hollow rectangular prism
driver_inertia_tensor = masses[7]*[((d_l_x/2)^2+2*d_w_y^2+d_h_z^2) 0 0; 0 2*((d_l_x/2)^2+d_h_z^2) 0; 0 0 ((d_l_x/2)^2+2*d_w_y^2+d_h_z^2)]/24 + masses[7]*[1 0 0; 0 2 0; 0 0 1]*d_l_x^2/32 # The human body is two thin plates bent at the hip
global position_dependent_inertia_tensor = zeros(3, 3)  # Initialize the inertia tensor that accouts for inertia from the different positions of each individual part

for i in 1:length(masses)
    a2 = ((pos_x2[i])^2 + (pos_y2[i])^2 + (pos_z2[i])^2) * [1 0 0; 0 1 0; 0 0 1]
    b2 = [pos_x2[i], pos_y2[i], pos_z2[i]] * [pos_x2[i], pos_y2[i], pos_z2[i]]'
    global position_dependent_inertia_tensor += masses[i] * (a2 - b2)
end

overall_inertia_tensor = position_dependent_inertia_tensor + 4*tire_inertia_tensor + accumulator_inertia_tensor + body_inertia_tensor + driver_inertia_tensor

## Display Results

println("Center of Mass (in meters) is at ", [pos_x_ave, pos_y_ave, pos_z_ave])
println("Inertia Tensor (in kg*m^2) about CM is ", overall_inertia_tensor)