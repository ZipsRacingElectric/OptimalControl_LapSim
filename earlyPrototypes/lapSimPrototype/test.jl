using JuMP
import Ipopt
import Plots

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

model = Model(Ipopt.Optimizer)
set_silent(model)

# Create State and Control Variables
@variable(model, x_v[1:T] >= 0, start = v_0)           # Velocity
@variable(model, x_h[1:T] >= 0, start = h_0)           # Height
@variable(model, x_m[1:T] >= m_T, start = m_0)         # Mass
@variable(model, 0 <= u_t[1:T] <= u_t_max, start = 0); # Thrust


@variable(model, Δt>=eps(), start = .2/T)


# Boundary Conditions
fix(x_v[1], v_0; force = true)
fix(x_h[1], h_0; force = true)
fix(x_m[1], m_0; force = true)
fix(u_t[T], 0.0; force = true)

    # attempt: minimize time to final height h = 1.011
    fix(x_h[T], h_f; force = true)

# Objective Function
#@objective(model,Max, x_h[T])

@objective(model,Min,Δt)

# Forces: defined as functions
    # also note, these are normalized or non-dimensionalized somehow/somewhere
D(x_h, x_v) = D_c * x_v^2 * exp(-h_c * (x_h - h_0) / h_0)       # drag as function of lots of stuff
g(x_h) = g_0 * (h_0 / x_h)^2                                    # gravity as function of height

# Dynamics: implemented as constraints
ddt(x::Vector, t::Int) = (x[t] - x[t-1]) / Δt
@constraint(model, [t in 2:T], ddt(x_h, t) == x_v[t-1])
@constraint(
    model,
    [t in 2:T],
    ddt(x_v, t) == (u_t[t-1] - D(x_h[t-1], x_v[t-1])) / x_m[t-1] - g(x_h[t-1]),
)
@constraint(model, [t in 2:T], ddt(x_m, t) == -u_t[t-1] / c);


#@constraint(model, x_h(T)==1.011)

print(model)

optimize!(model)
assert_is_solved_and_feasible(model)
print(solution_summary(model))

tf = value.(T)*value.(Δt)

# Plot solution:
function plot_trajectory(y; kwargs...)
    return Plots.plot(
        (1:T) * value.(Δt),
        value.(y);
        xlabel = "Time (s)",
        legend = false,
        kwargs...,

    )
end

display(Plots.plot(
    plot_trajectory(x_h; ylabel = "Altitude"),
    plot_trajectory(x_m; ylabel = "Mass"),
    plot_trajectory(x_v; ylabel = "Velocity"),
    plot_trajectory(u_t; ylabel = "Thrust");
    layout = (2, 2),
))
    print("final time is: $(tf)")