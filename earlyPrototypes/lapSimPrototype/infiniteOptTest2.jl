using InfiniteOpt, Ipopt, Plots

model = InfiniteModel(optimizer_with_attributes(Ipopt.Optimizer, "print_level" => 0));

@infinite_parameter(model,τ in [0,1],num_supports = 1001,)

h_0 = 1                      # Initial height
v_0 = 0                      # Initial velocity
m_0 = 1.0                    # Initial mass
m_f = 0.6                    # Final mass
g_0 = 1                      # Gravity at the surface
h_c = 500                    # Used for drag
c = 0.5 * sqrt(g_0 * h_0)    # Thrust-to-fuel mass
D_c = 0.5 * 620 * m_0 / g_0  # Drag scaling
u_t_max = 3.5 * g_0 * m_0    # Maximum thrust
T_max = 0.2                  # Number of seconds
h_f = 1.011

@variables(model, begin
    # state variables
    h, Infinite(τ), (start = h_0)
    v, Infinite(τ), (start = v_0)
    m >= m_f, Infinite(τ), (start = m_0)
    # control variables
    0 <= u <= u_t_max, Infinite(τ), (start = 0)
end)

@variable(model, 0.0000000001 <= tf <= 100, start = 1)

@objective(model,Min,tf)

# functions for dynamics
D(h, v) = D_c * v^2 * exp(-h_c * (h - h_0) / h_0)           # drag as function of lots of stuff
g(h) = g_0 * (h_0 / h)^2                                    # gravity as function of height

@register(model,D(h,v))
@register(model,g(h))

# Dynamics
@constraint(model, ∂(h,τ) == tf*(v))
@constraint(model, [i = 1:1], ∂(v,τ) == tf*((u - D(h, v) )/m -g(h)))
@constraint(model, [i = 1:1], ∂(m, τ) == tf * (-u / c))


# Initial Conditions
@constraint(model, h(0) == h_0)
@constraint(model, v(0) == v_0)
@constraint(model, m(0) == m_0)

# Set terminal constraints
@constraint(model, h(1) == h_f)
#@constraint(model, u(1) == 0)


#print(model)

optimize!(model)
print(solution_summary(optimizer_model(model)))

opt_u = value(u)
opt_h = value(h)
opt_m = value(m)
opt_v = value(v)
opt_tf = value(tf)

# Get the scaled times
ts = value(τ) * opt_tf

display(plot(ts, [opt_h opt_v opt_m opt_u], layout = (2, 2), legend = false, 
     xlabel = "Time", ylabel = ["Position" "Velocity" "Mass" "Force"],
     xlims = (0, opt_tf)))


print("final time is: $(opt_tf)")