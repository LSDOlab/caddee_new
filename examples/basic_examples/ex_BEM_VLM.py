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
from caddee import GEOMETRY_FILES_FOLDER
import time 
from lsdo_rotor import BEM, BEMParameters
from VAST.core.fluid_problem import FluidProblem
from VAST.core.vast_solver import VASTFluidSover


# ------------------------ Importing, refitting and plotting the geometry (plotting is optional)
geometry = lg.import_geometry(GEOMETRY_FILES_FOLDER  / 'test_prop_plus_wing.stp', parallelize=False)
geometry.refit(parallelize=False, num_coefficients=(10, 10))
# geometry.plot()

# ------------------------ Declaring components 
#    'component_name=' is a user-defined string for how they want to name a component
#    'b_spline_search_name=' is a list of the b-spline patches that make up a component as defined by the OpenVSP geometry
prop_disk = geometry.declare_component(component_name='propeller_disk', b_spline_search_names=['Disk'])
wing = geometry.declare_component(component_name='wing', b_spline_search_names=['WingGeom'])

# ------------------------ Creating a VLM mesh with 25-1 span-wise and 15-1 chord-wise panels
num_spanwise_vlm = 25
num_chordwise_vlm = 15

# Calling a helper function to create VLM mesh based on the 4 corner points of 
# the wing plus the center point of the trailing edge
wing_mesh = cd.make_vlm_camber_mesh(
    geometry=geometry,
    wing_component=wing,
    num_spanwise=num_spanwise_vlm,
    num_chordwise=num_chordwise_vlm,
    le_right=np.array([3, 5., 0.]),
    le_left=np.array([3., -5., 0.]),
    te_right=np.array([4.33, 5., 0.]),
    te_center=np.array([5., 0., 0.]),
    te_left=np.array([4.33, -5., 0.]),
    plot=False
)


# ------------------------ Creating the 'mesh' for BEM analysis (i.e., get the thrust vector and origin)
# Calling helper function with 2 pairs of oppposite points on the disk edge (y1/2, z1/2)
# and the number of radial stations

num_radial = 30
rotor_mesh = cd.make_rotor_mesh(
    geometry=geometry,
    disk_component=prop_disk,
    origin=np.array([0., 0., 0.,]),
    y2=np.array([0., -1., 0.]),
    y1=np.array([0., 1., 0.]),
    z1=np.array([0., 0., -1.]),
    z2=np.array([0., 0., 1.]),
    plot=False,
    num_radial=num_radial,
)

# Make sure the thrust vector points in the correct direction according to the body-fixed reference frame
# e.g., [1, 0, 0] means in the direction of the nose of the aircraft 
print(rotor_mesh.thrust_vector)
print(rotor_mesh.radius)


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
pitch_angle = m3l_model.create_input('pitch_angle', val=np.array([np.deg2rad(0)]), dv_flag=True, lower=np.deg2rad(-10), upper=np.deg2rad(10))
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
        (1, ) + wing_mesh.shape[1:]
    ],
    fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake', symmetry=True),
    mesh_unit='m',
    cl0=[0.25]
)

# Evaluate VLM outputs and register them as outputs
vlm_outputs = vlm_model.evaluate(
    ac_states=ac_states,
    meshes=[wing_mesh],
)

m3l_model.register_output(vlm_outputs)


# prop forces and moments
# Create BEMParameters and BEM objects
bem_parameters = BEMParameters(
    num_blades=3,
    num_radial=num_radial,
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
chord_profile = m3l_model.create_input('chord', val=np.linspace(0.3, 0.1, num_radial))
twist_profile = m3l_model.create_input('twist', val=np.deg2rad(np.linspace(60, 10, num_radial)))
# NOTE: 'chord_profile' and 'twist_profile' can also be created using the rotor blade geometry in combination with projections

# Evaluate and register BEM outputs 
bem_outputs = bem_model.evaluate(
    ac_states=ac_states, rpm=omega, rotor_radius=rotor_mesh.radius, 
    thrust_vector=rotor_mesh.thrust_vector, thrust_origin=rotor_mesh.thrust_origin, 
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
    num_spanwise_vlm = 24
    num_chordwise_vlm = 14

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

