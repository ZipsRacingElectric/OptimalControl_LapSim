using InfiniteOpt, Ipopt, Plots

model = InfiniteModel(Ipopt.Optimizer)


h_0 = 1                      # Initial height
v_0 = 0                      # Initial velocity
m_0 = 1.0                    # Initial mass
m_T = 0.6                    # Final mass
g_0 = 1                      # Gravity at the surface
h_c = 500                    # Used for drag
c = 0.5 * sqrt(g_0 * h_0)    # Thrust-to-fuel mass
D_c = 0.5 * 620 * m_0 / g_0  # Drag scaling
u_t_max = 3.5 * g_0 * m_0    # Maximum thrust
T_max = 0.2                  # Number of seconds
T = 1_000                    # Number of time steps
#Δt = 0.2 / T;                # Time per discretized step
h_f = 1.011

# Infinite parameter: in this case, τ = t/tf, t is ??, tf is final time
@infinite_parameter(model, τ in [0,1], num_supports = 101)
set_silent(model)


# Create the variables
@variable(model, 0 <= u <= u_t_max, Infinite(τ), start = 0)
@variable(model, h, Infinite(τ), start = 0)
@variable(model, 0 <= v, Infinite(τ), start = 0)
@variable(model, m_0 >= m >= m_T, Infinite(τ), start = m_0)
@variable(model, 0.0000000001 <= tf <= 100, start = 1)

# Set the objective
@objective(model, Min, tf)

# functions for Forces
D(h, v) = D_c * v^2 * exp(-h_c * (h - h_0) / h_0)       # drag as function of lots of stuff
g(h) = g_0 * (h_0 / h)^2                                    # gravity as function of height

# nope, didn't work, these take infinite parameters as functions
    # try to implement as parameter function??
    #@parameter_function(model,D == D_c * v^2 * exp(-h_c * (h - h_0) / h_0))
    #@parameter_function(model,g == g_0 * (h_0 / h)^2)

# Define the ODEs
@constraint(model, ∂(h, τ) == tf * v)                       


@constraint(model, m * ∂(v, τ) == tf * (u - 0.2 * v^2))     # f = ma
#@constraint(model, m * ∂(v, τ) == tf * (u - D_c * v^2 * exp(-h_c * (1 - h_0) / h_0)))
#@constraint(model, m*∂(v, τ) == tf*(u - D(h, v) ))

#@constraint(model, ∂(m, τ) == tf * (-0.01 * u^2))           # mass burn
@constraint(model, ∂(m, τ) == tf * (-u / c));

# Set the initial conditions
@constraint(model, h(0) == h_0)
@constraint(model, v(0) == v_0)
@constraint(model, m(0) == m_0)

# Set terminal constraints
@constraint(model, h(1) == h_f)
@constraint(model, u(1) == 0)

print(model)
# Optimize and get the results
optimize!(model)

isWorky = termination_status(model)
print(isWorky)

opt_u = value(u)
opt_h = value(h)
opt_m = value(m)
opt_v = value(v)
opt_tf = value(tf)


# Get the scaled times
ts = value(τ) * opt_tf

# Plot the optimal trajectories
plot(ts, [opt_h opt_v opt_m opt_u], layout = (4, 1), legend = false, 
     xlabel = "Time", ylabel = ["Position" "Velocity" "Mass" "Force"],
     xlims = (0, opt_tf))



