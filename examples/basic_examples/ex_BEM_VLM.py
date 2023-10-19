'''Example 3 : BEM + VLM with simple geometry

This example illustrates how to use a simple OpenVSP geometry to create meshes for VLM and BEM analysis
and use CADDEE to evaluate the VLM and BEM solvers based on a simple steady design condition.
'''
import numpy as np
import caddee.api as cd
import m3l
from python_csdl_backend import Simulator
from modopt.scipy_library import SLSQP
from modopt.csdl_library import CSDLProblem
import lsdo_geo as lg
from caddee import IMPORTS_FILES_FOLDER
import time 
from lsdo_rotor import BEM, BEMParameters
from VAST.core.fluid_problem import FluidProblem
from VAST.core.vast_solver import VASTFluidSover


# ------------------------ Importing, refitting and plotting the geometry (plotting is optional)
geometry = lg.import_geometry(IMPORTS_FILES_FOLDER  / 'test_prop_plus_wing_2.stp', parallelize=False)
geometry.refit(parallelize=True, num_coefficients=(10, 10))
geometry.plot()

# ------------------------ Declaring components 
#    'component_name=' is a user-defined string for how they want to name a component
#    'b_spline_search_name=' is a list of the b-spline patches that make up a component as defined by the OpenVSP geometry
prop_disk = geometry.declare_component(component_name='propeller_disk', b_spline_search_names=['Disk'])
wing = geometry.declare_component(component_name='wing', b_spline_search_names=['WingGeom'])

# ------------------------ Creating a VLM mesh with 15-1 span-wise and 10-1 chord-wise panels
num_spanwise_vlm = 15
num_chordwise_vlm = 10

# Projecting a linearly spaced array onto the leading and trailing edge
# NOTE: project returns parametric coordinate (i.e., u, v) 
wing_le_parametric = wing.project(np.linspace(np.array([3., -5., 0.]), np.array([3., 5., 0.]), num_spanwise_vlm), plot=False)
wing_te_parametric = wing.project(np.linspace(np.array([4.33+1, -5., 0.]), np.array([4.33+1, 5., 0.]), num_spanwise_vlm), plot=False)

# NOTE: go from parametric coordinates to physical coordinates by calling 'geometry.evaluate()' as below
wing_le_coord = geometry.evaluate(wing_le_parametric).reshape((-1, 3))
wing_te_coord = geometry.evaluate(wing_te_parametric).reshape((-1, 3))

# A user can print the values of m3l variable by calling '.value'
print(wing_le_coord.value)
print(wing_te_coord.value)

# Getting a linearly spaced (2-D) array between the leading and trailing edge
wing_chord = m3l.linspace(wing_le_coord, wing_te_coord, num_chordwise_vlm)

# Projecting the 2-D array onto the upper and lower surface of the wing to get the camber surface mesh
wing_upper_surface_wireframe_parametric = wing.project(wing_chord.value + np.array([0., 0., 1.]), direction=np.array([0., 0., 1.]), grid_search_density_parameter=25, plot=False)
wing_lower_surface_wireframe_parametric = wing.project(wing_chord.value - np.array([0., 0., 1.]), direction=np.array([0., 0., -1.]), grid_search_density_parameter=25, plot=False)
wing_upper_surface_wireframe = geometry.evaluate(wing_upper_surface_wireframe_parametric).reshape((num_chordwise_vlm, num_spanwise_vlm, 3))
wing_lower_surface_wireframe = geometry.evaluate(wing_lower_surface_wireframe_parametric).reshape((num_chordwise_vlm, num_spanwise_vlm, 3))

wing_camber_surface = m3l.linspace(wing_upper_surface_wireframe, wing_lower_surface_wireframe, 1)#.reshape((-1, 3))

# Optionally, the resulting camber surface mesh can be plotted
geometry.plot_meshes(meshes=wing_camber_surface, mesh_plot_types=['wireframe'], mesh_opacity=1., mesh_color='#F5F0E6')


# ------------------------ Creating the 'mesh' for BEM analysis (i.e., get the thrust vector and origin)
# Define projections for the thrust origin and for oppositely spaced point on the rotor disk edge
prop_disk_origin_parametric = prop_disk.project(np.array([0., 0., 0.,]), plot=False)
disk_edge_point_1_parametric = prop_disk.project(np.array([0., -1., 0.]), plot=False)
disk_edge_point_2_parametric = prop_disk.project(np.array([0., 1., 0.]), plot=False)
disk_edge_point_3_parametric = prop_disk.project(np.array([0., 0., -1.]), plot=False)
disk_edge_point_4_parametric = prop_disk.project(np.array([0., 0., 1.]), plot=False)

prop_disk_origin = geometry.evaluate(prop_disk_origin_parametric)
disk_edge_point_1 = geometry.evaluate(disk_edge_point_1_parametric)
disk_edge_point_2 = geometry.evaluate(disk_edge_point_2_parametric)
disk_edge_point_3 = geometry.evaluate(disk_edge_point_3_parametric)
disk_edge_point_4 = geometry.evaluate(disk_edge_point_4_parametric)

# Compute the rotor_radius and thrust vector
rotor_radius = m3l.norm(disk_edge_point_2 - disk_edge_point_1) / 2
thrust_vector = m3l.cross(disk_edge_point_3 - disk_edge_point_4,disk_edge_point_2-disk_edge_point_1)
# NOTE the thrust vector passed into BEM needs to be a unit vector pointing in the correct direction
thrust_unit_vector = thrust_vector / m3l.norm(thrust_vector)
print(thrust_vector.value)
print(thrust_unit_vector.value) 
# Make sure the thrust vector points in the correct direction according to the body-fixed reference frame
# e.g., [1, 0, 0] means in the direction of the nose of the aircraft 

# ------------------------ Analaysis
# Create caddee and SystemModel object
caddee = cd.CADDEE()
caddee.system_model = system_model = cd.SystemModel()

# create m3l system model
m3l_model = m3l.Model()


# cruise condition
cruise_condition = cd.CruiseCondition(
    name="cruise_1",
    stability_flag=False,
    num_nodes=1,
)
# Set operating conditions for steady design condition
mach_number = m3l_model.create_input('mach_number', val=np.array([0.2]))
altitude = m3l_model.create_input('cruise_altitude', val=np.array([1500]))
pitch_angle = m3l_model.create_input('pitch_angle', val=np.array([np.deg2rad(2.67324908)]), dv_flag=True, lower=np.deg2rad(-10), upper=np.deg2rad(10))
range = m3l_model.create_input('cruise_range', val=np.array([40000]))

# Evaluate aircraft states as well as atmospheric properties based on inputs to operating condition
ac_states, atmosphere = cruise_condition.evaluate(
    mach_number=mach_number, 
    pitch_angle=pitch_angle, 
    altitude=altitude, 
    cruise_range=range
)
m3l_model.register_output(ac_states)
m3l_model.register_output(atmosphere)

# aero forces and moments
vlm_model = VASTFluidSover(
    name='cruise_vlm_model',
    surface_names=[
        'wing_vlm_mesh'
    ],
    surface_shapes=[
        (1, ) + wing_camber_surface.shape[1:]
    ],
    fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
    mesh_unit='m',
    cl0=[0.25]
)

# Evaluate VLM outputs and register them as outputs
vlm_outputs = vlm_model.evaluate(
    ac_states=ac_states,
    meshes=[wing_camber_surface],
)

m3l_model.register_output(vlm_outputs)


# prop forces and moments
# Create BEMParameters and BEM objects
bem_parameters = BEMParameters(
    num_blades=3,
    num_radial=25,
    num_tangential=1,
    airfoil='NACA_4412',
    use_custom_airfoil_ml=True,
)

bem_model = BEM(
    name='cruise_bem',
    BEM_parameters=bem_parameters,
    num_nodes=1,
)

# Create necessary m3l variables as inputs in BEM 
omega = m3l_model.create_input('omega', val=np.array([2109.07445251]), dv_flag=True, lower=2000, upper=2800, scaler=1e-3)
chord_profile = m3l_model.create_input('chord', val=np.linspace(0.3, 0.1, 25))
twist_profile = m3l_model.create_input('twist', val=np.deg2rad(np.linspace(60, 10, 25)))
# NOTE: 'chord_profile' and 'twist_profile' can also be created using the rotor blade geometry in combination with projections

# Evaluate and register BEM outputs 
bem_outputs = bem_model.evaluate(
    ac_states=ac_states, rpm=omega, rotor_radius=rotor_radius, 
    thrust_vector=thrust_unit_vector, thrust_origin=prop_disk_origin, 
    atmosphere=atmosphere, blade_chord=chord_profile, blade_twist=twist_profile
)
m3l_model.register_output(bem_outputs)

# Assemble caddee csdl model
caddee_csdl_model = m3l_model.assemble_csdl()

# create and run simulator
sim = Simulator(caddee_csdl_model, analytics=True)
sim.run()

# Optionally, a user can print all variables and their values that were registered as outputs in this run file
cd.print_caddee_outputs(m3l_model, sim)

# Optional for advanced users: A user can take advantage of CADDEE's SIFR interface to plot field quantities such as pressure 
# on top of the geometry 
plot = True

if plot:
    # Here, we need to define where VLM outputs exist on the geometry
    # There are 15 and 10 span-wise and chord-wise nodes, respectively, which means that there are 14 and 9 panels
    num_spanwise_vlm = 14
    num_chordwise_vlm = 9

    # We prject the points where we expect VLM outputs onto the geometry
    wing_le_parametric = wing.project(np.linspace(np.array([3., -5., 0.]), np.array([3., 5., 0.]), num_spanwise_vlm), plot=False)
    wing_te_parametric = wing.project(np.linspace(np.array([4.33+1, -5., 0.]), np.array([4.33+1, 5., 0.]), num_spanwise_vlm), plot=False)

    wing_le_coord = geometry.evaluate(wing_le_parametric).reshape((-1, 3))
    wing_te_coord = geometry.evaluate(wing_te_parametric).reshape((-1, 3))

    wing_chord = m3l.linspace(wing_le_coord, wing_te_coord, num_chordwise_vlm)

    wing_upper_surface_wireframe_parametric = wing.project(wing_chord.value + np.array([0., 0., 1.]), direction=np.array([0., 0., 1.]), grid_search_density_parameter=25, plot=False)
    wing_lower_surface_wireframe_parametric = wing.project(wing_chord.value - np.array([0., 0., 1.]), direction=np.array([0., 0., -1.]), grid_search_density_parameter=25, plot=False)


    force_z = sim['cruise_vlm_model.wing_vlm_mesh_total_forces'][:, :, 2]
    force_stack = np.vstack((force_z, force_z))

    # Create a function space for the quantity of interest
    wing_lift_space = geometry.space.create_sub_space(sub_space_name='wing_lift_space', b_spline_names=wing.b_spline_names)
    
    # Fit a b-spline based on the VLM data by solving a least-squares problem to find the control points (i.e., coefficients)
    pressure_coefficients = wing_lift_space.fit_b_spline_set(fitting_points=force_stack.reshape((-1,1)), fitting_parametric_coordinates=wing_upper_surface_wireframe_parametric + wing_lower_surface_wireframe_parametric, regularization_parameter=1e-2)

    # Create a function from the function space
    wing_pressure_function = wing_lift_space.create_function(name='left_wing_pressure_function', 
                                                                        coefficients=pressure_coefficients, num_physical_dimensions=1)

    # Plot
    wing.plot(color=wing_pressure_function)

