'''Example 1 : TC 1 analysis and optimization script

This advanced example demonstrates how to build the analysis script for the complex NASA ULI technical 
challenge 1 (TC1) problem, which modls three steady design conditions: Hover, Climb and Cruise. We use 
a combination of low-fidelity pyhscisc-based models for the aerodynamics, semi-empirical, and regression
models for the analysis. 

The central geometry along with meshes required for the analysis are imported from 'ex_lpc_geom.py'.
'''


# Module imports
import numpy as np 
import caddee.api as cd
from lsdo_rotor import BEMParameters, evaluate_multiple_BEM_models, BEM
from VAST import FluidProblem, VASTFluidSover
from lsdo_motor import evaluate_multiple_motor_sizing_models, evaluate_multiple_motor_analysis_models, MotorAnalysis, MotorSizing

# Imports from geometry setup file
from ex_lpc_geom import system_model, RotorMeshes


bem_hover_rotor_parameters = BEMParameters(
    num_blades=2,
    num_radial=30,
    num_tangential=30,
    airfoil='NACA_4412',
    use_custom_airfoil_ml=False,
    mesh_units='ft',
)

bem_hover_rotor_parameters = BEMParameters(
    num_blades=4,
    num_radial=30,
    num_tangential=1,
    airfoil='NACA_4412',
    use_custom_airfoil_ml=False,
    mesh_units='ft',
)


# region sizing
motor_diameters = []
motor_lengths = []
for i in range(num_rotors + 1):
    motor_diameters.append(system_model.create_input(f'motor_diameter_{i}', val=0.17, dv_flag=motor, upper=0.25, lower=0.05, scaler=2))
    motor_lengths.append(system_model.create_input(f'motor_length_{i}', val=0.1, dv_flag=motor, upper=0.15, lower=0.05, scaler=2))

motor_mass_properties = evaluate_multiple_motor_sizing_models(
    motor_diameter_list=motor_diameters,
    motor_length_list=motor_lengths,
    motor_origin_list=origin_list_plus_pusher,
    name_prefix='motor_sizing',
    m3l_model=system_model,
)