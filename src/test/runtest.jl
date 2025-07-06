using Test
include("..\\vehicleParameters\\CarParametersModule.jl")
using .CarParametersModule: update_parameters, reset_parameters, create_car


spreadsheetPath = "C:\\Users\\benmo\\Documents\\GitHub\\Vehicle-Dynamics\\MATLAB\\vehicle_data\\zr25_data.xlsx"

@testset "Car parameter update and reset" begin
    # Create a dummy car from test spreadsheet
    car = create_car(spreadsheetPath)  # Make sure this is small & controlled "test/test_parameters.xlsx"

    # Store original values
    original_mass = car.mass_total
    original_track_width_front = car.track_width_front

    original_lltd = car.lltd_front

    # === Step 1: Modify values ===
    new_mass = car.vehicle_mass + 50.0
    new_track_width_front = original_track_width_front + 0.1

    car.vehicle_mass = new_mass
    car.track_width_front = new_track_width_front

    update_parameters(car)

    # === Step 2: Test if values were updated ===
    @test car.mass_total == new_mass + car.driver_mass
    @test car.track_width_front == new_track_width_front
    @test car.lltd_front != original_lltd

    # === Step 3: Reset to original ===
    reset_parameters(car)

    # === Step 4: Test if values were reset ===
    @test car.mass_total ≈ original_mass atol=1e-8
    @test car.track_width_front ≈ original_track_width_front atol=1e-8
    @test car.lltd_front ≈ original_lltd atol=1e-8
end
