'''Example 3 : BEM + VLM with simple geometry'''
import numpy as np
import caddee.api as cd
import m3l
from python_csdl_backend import Simulator
from modopt.scipy_library import SLSQP
from modopt.csdl_library import CSDLProblem
import lsdo_geo as lg
from caddee import GEOMETRY_FILES_FOLDER
import time 
from lsdo_rotor.core.BEM.BEM_caddee import BEM, BEMParameters
from VAST.core.fluid_problem import FluidProblem
from VAST.core.vast_solver import VASTFluidSover


# Importing, refitting and plotting the geometry (plotting is optional)
geometry = lg.import_geometry(GEOMETRY_FILES_FOLDER  / 'test_prop_plus_wing.stp', parallelize=False)
geometry.refit(parallelize=True, num_coefficients=(10, 10))
geometry.plot()

# Declaring components 
#       'component_name=' is a user-defined string for how they want to name a component
#       'b_spline_search_name=' is a list of the b-spline patches that make up a component as defined by the OpenVSP geometry
prop_disk = geometry.declare_component(component_name='propeller_disk', b_spline_search_names=['Disk'])
wing = geometry.declare_component(component_name='wing', b_spline_search_names=['WingGeom'])
# wing_upper_and_lower = geometry.declare_component(component_name='wing_upper_and_lower', b_spline_search_names=['WingGeom, 0, 36', 'WingGeom, 0, 37', 'WingGeom, 1, 42', 'WingGeom, 1, 43'])
# wing_upper = geometry.declare_component(component_name='wing_upper_and_lower', b_spline_search_names=['WingGeom, 0, 37',  'WingGeom, 1, 43'])

num_spanwise_vlm = 15
num_chordwise_vlm = 10

wing_le_parametric = wing.project(np.linspace(np.array([3., -5., 0.]), np.array([3., 5., 0.]), num_spanwise_vlm), plot=False)
wing_te_parametric = wing.project(np.linspace(np.array([4.33+1, -5., 0.]), np.array([4.33+1, 5., 0.]), num_spanwise_vlm), plot=False)

wing_le_coord = geometry.evaluate(wing_le_parametric).reshape((-1, 3))
wing_te_coord = geometry.evaluate(wing_te_parametric).reshape((-1, 3))
# print(wing_le_coord.value)
# print(wing_te_coord.value)

wing_chord = m3l.linspace(wing_le_coord, wing_te_coord, num_chordwise_vlm)
print(wing_chord.value)
print(wing_chord.shape)

wing_upper_surface_wireframe_parametric = wing.project(wing_chord.value + np.array([0., 0., 1.]), direction=np.array([0., 0., 1.]), grid_search_density_parameter=25, plot=False)
wing_lower_surface_wireframe_parametric = wing.project(wing_chord.value - np.array([0., 0., 1.]), direction=np.array([0., 0., -1.]), grid_search_density_parameter=25, plot=False)
wing_upper_surface_wireframe = geometry.evaluate(wing_upper_surface_wireframe_parametric).reshape((num_chordwise_vlm, num_spanwise_vlm, 3))
wing_lower_surface_wireframe = geometry.evaluate(wing_lower_surface_wireframe_parametric).reshape((num_chordwise_vlm, num_spanwise_vlm, 3))

print(wing_upper_surface_wireframe.shape)

wing_camber_surface = m3l.linspace(wing_upper_surface_wireframe, wing_lower_surface_wireframe, 1)#.reshape((-1, 3))
# geometry.plot_meshes(meshes=wing_camber_surface, mesh_plot_types=['wireframe'], mesh_opacity=1., mesh_color='#F5F0E6')
# print(wing_camber_surface.value)
# print(wing_camber_surface.shape)
# 

# prop_disk.plot()
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



rotor_radius = m3l.norm(disk_edge_point_2 - disk_edge_point_1) / 2
print(rotor_radius.shape)
print(rotor_radius.value)
thrust_vector = m3l.cross(disk_edge_point_3 - disk_edge_point_4,disk_edge_point_2-disk_edge_point_1)
print(thrust_vector.value)
print(thrust_vector.shape)
thrust_unit_vector = thrust_vector / m3l.norm(thrust_vector)
print(m3l.norm(thrust_vector).shape)
print(thrust_unit_vector.value)


bem_parameters = BEMParameters(
    num_blades=3,
    num_radial=25,
    num_tangential=1,
    airfoil='NACA_4412',
    use_custom_airfoil_ml=True,
)

caddee = cd.CADDEE()
caddee.system_model = system_model = cd.SystemModel()

# m3l model
m3l_model = m3l.Model()


# cruise condition
cruise_condition = cd.CruiseCondition(
    name="cruise_1",
    stability_flag=False,
    num_nodes=1,
)

mach_number = m3l_model.create_input('mach_number', val=np.array([0.2]))
altitude = m3l_model.create_input('cruise_altitude', val=np.array([1500]))
pitch_angle = m3l_model.create_input('pitch_angle', val=np.array([np.deg2rad(2.67324908)]), dv_flag=True, lower=np.deg2rad(-10), upper=np.deg2rad(10))
range = m3l_model.create_input('cruise_range', val=np.array([40000]))

ac_states, atmosphere = cruise_condition.evaluate(
    mach_number=mach_number, 
    pitch_angle=pitch_angle, 
    altitude=altitude, 
    cruise_range=range
)
m3l_model.register_output(ac_states)

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


vlm_outputs = vlm_model.evaluate(
    atmosphere=atmosphere,
    ac_states=ac_states,
    meshes=[wing_camber_surface],
)

m3l_model.register_output(vlm_outputs)


# prop forces and moments
bem_model = BEM(
    name='cruise_bem',
    BEM_parameters=bem_parameters,
    num_nodes=1,
)

omega = m3l_model.create_input('omega', val=np.array([2109.07445251]), dv_flag=True, lower=2000, upper=2800, scaler=1e-3)
chord_profile = m3l_model.create_input('chord', val=np.linspace(0.3, 0.1, 25))
twist_profile = m3l_model.create_input('twist', val=np.deg2rad(np.linspace(60, 10, 25)))

bem_outputs = bem_model.evaluate(
    ac_states=ac_states, rpm=omega, rotor_radius=rotor_radius, 
    thrust_vector=thrust_unit_vector, thrust_origin=prop_disk_origin, 
    atmosphere=atmosphere, blade_chord=chord_profile, blade_twist=twist_profile
)

m3l_model.register_output(bem_outputs)



caddee_csdl_model = m3l_model.assemble_csdl()
# caddee_csdl_model.add_objective('cruise_1_eom_model.accelerations')

# create and run simulator
sim = Simulator(caddee_csdl_model, analytics=True)
sim.run()

cd.print_caddee_outputs(m3l_model=m3l_model, sim=sim)

exit()
num_spanwise_vlm = 14
num_chordwise_vlm = 9

wing_le_parametric = wing.project(np.linspace(np.array([3., -5., 0.]), np.array([3., 5., 0.]), num_spanwise_vlm), plot=False)
wing_te_parametric = wing.project(np.linspace(np.array([4.33+1, -5., 0.]), np.array([4.33+1, 5., 0.]), num_spanwise_vlm), plot=False)

wing_le_coord = geometry.evaluate(wing_le_parametric).reshape((-1, 3))
wing_te_coord = geometry.evaluate(wing_te_parametric).reshape((-1, 3))

wing_chord = m3l.linspace(wing_le_coord, wing_te_coord, num_chordwise_vlm)

wing_upper_surface_wireframe_parametric = wing.project(wing_chord.value + np.array([0., 0., 1.]), direction=np.array([0., 0., 1.]), grid_search_density_parameter=25, plot=False)
wing_lower_surface_wireframe_parametric = wing.project(wing_chord.value - np.array([0., 0., 1.]), direction=np.array([0., 0., -1.]), grid_search_density_parameter=25, plot=False)


force_z = sim['cruise_vlm_model.wing_vlm_mesh_total_forces'][:, :, 2]
force_stack = np.vstack((force_z, force_z))

wing_lift_space = geometry.space.create_sub_space(sub_space_name='wing_lift_space', b_spline_names=wing.b_spline_names)
pressure_coefficients = wing_lift_space.fit_b_spline_set(fitting_points=force_stack.reshape((-1,1)), fitting_parametric_coordinates=wing_upper_surface_wireframe_parametric + wing_lower_surface_wireframe_parametric, regularization_parameter=1e-2)

wing_pressure_function = wing_lift_space.create_function(name='left_wing_pressure_function', 
                                                                       coefficients=pressure_coefficients, num_physical_dimensions=1)

wing.plot(color=wing_pressure_function)


cd.print_caddee_outputs(m3l_model, sim)

# print(sim['cruise_bem.BEM_external_inputs_model.propeller_radius'])
# print(sim['cruise_bem.chord_profile'])
