# OptimalControl_LapSim
 Repository for Zips Racing's Optimal Control Based Laptime Simulator, developed by Ben Model and Kenneth Dubos for our Capstone Project

    This project uses optimal control algorithms from InfiniteOpt (a JuMP extension) and the IPOPT Solver to provide an objective tool for analyzing high level race car design tradeoffs. Optimal Control uses systems of (usually) differential algebraic equations as the constraint equations in a (most often) non-linear optimization problem. In our case, the objective function minimizes the time taken by the car to complete a lap, without imposing a predefined path to take. (i.e. a minimum final time, free path formulation).


    turnyTurn.jl is the first working prototype of a turning model. It implements the dynamics of the car relative to a Frenet-Serret reference frame (TNB Frame) that follows the centerline of the track, allowing a "1-D" parameterization by time and arc length s(t) along the track. Position is always relative to the TNB Frame, attached to the centerline, following the car down the track. The actual dynamics are rather arbitrary, mass for instance is 1, drag and downforce are something like .3v^2. But everything acts close to what one may expect from first principles.

    vehicleParameters contains a julia file that defines a struct to contain all of the parameters needed to describe the car's performance. It is adapted from a matlab object I wrote that does the same thing, but without the object-oriented-ness.

    Next steps will be integrating these two and implementing actual race car dynamics, based on a simple (very simple) tire model. This will include weight transfers on each tire, probably using LLTD's rather than the simpler CG moment type stuff, as these can more accurately (and in a way more simply) represent the balance/handling of the car. For now we will keep the track simple, but effort needs to go into getting track data (being worked on by Kenneth) into the problem as well. 
    There is also lots of "infrastructure" to build, such as functions for translating/plotting the data in a useable form, a more modular and extensible main script, that hides some of the esoterics and allows a user to play with values they might care more about, etc. There's also the whole issue of fitting this model to the actual car, but the actual car might be further behind than this :/


TO RUN:
(straight up, these instructions are from chatGPT, Ken you'll have to beta test me here, sorry bro)

git clone https://github.com/your-repo/OptimalControl_LapSim.git
cd OptimalControl_LapSim
julia --project=@. -e 'using Pkg; Pkg.instantiate()'



