# Inertia Tensor With 4 Wheels and Battery

a = 0.7946 # m
b = 0.7344 # m
htf = 1230/1000/2 # m
htr = 1210/1000/2 # m
r = 0.259 # m

masses = [19.1, 19.1, 19.1, 19.1, 60] # lbm
pos_x = [a, a, -b, -b, -0.465]
pos_y = [htf, -htf, htr, -htr, 0]
pos_z = [r, r, r, r, 0.238]

pos_x_ave = sum(pos_x) / length(pos_x)
pos_y_ave = sum(pos_y) / length(pos_y)
pos_z_ave = sum(pos_z) / length(pos_z)

pos_x2 = pos_x .- pos_x_ave
pos_y2 = pos_y .- pos_y_ave
pos_z2 = pos_z .- pos_z_ave

global inertia_tensor = zeros(3, 3)  # Initialize the inertia tensor

for i in 1:length(masses)
    a2 = ((pos_x2[i])^2 + (pos_y2[i])^2 + (pos_z2[i])^2) * [1 0 0; 0 1 0; 0 0 1]
    b2 = [pos_x2[i], pos_y2[i], pos_z2[i]] * [pos_x2[i], pos_y2[i], pos_z2[i]]'
    global inertia_tensor += masses[i] * (a2 - b2)
end

println("Center of Mass (in meters) is at ", [pos_x_ave, pos_y_ave, pos_z_ave])
println("Inertia Tensor (in lbm*m^2) about CM is ", inertia_tensor)