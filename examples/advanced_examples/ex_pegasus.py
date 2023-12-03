'''Example 2 : Description of example 2'''
import caddee.api as cd
from caddee import STP_FILES_FOLDER
import csdl
from python_csdl_backend import Simulator
from aframe.core.beam_module import LinearBeam, LinearBeamMesh
from modopt.scipy_library import SLSQP
from modopt.csdl_library import CSDLProblem
import csdl
import numpy as np
import array_mapper as am
import scipy.io as sio
import m3l
import lsdo_geo as lg


loads_dict = sio.loadmat('caddee_execution_scripts/pegasus/loads_2p5g_n1g_aero_static.mat')
static_forces = loads_dict['forces']*4.44822162
static_moments = loads_dict['moments']*0.11298482933333
num_nodes = len(static_forces[0,:,0])
forces, moments = np.zeros((num_nodes,3)), np.zeros((num_nodes,3))
for i in range(num_nodes - 2):
    forces[i+2,:] = static_forces[0,i,:]
    moments[i+2,:] = static_moments[0,i,:]

left_forces = np.flip(forces, axis=0)
right_forces = 1*forces
left_moments = np.flip(moments, axis=0)
right_moments = 1*moments




caddee = cd.CADDEE()
caddee.system_representation = sys_rep = cd.SystemRepresentation()
caddee.system_parameterization = sys_param = cd.SystemParameterization(system_representation=sys_rep)

file_name = 'pegasus.stp'

spatial_rep = sys_rep.spatial_representation
spatial_rep.import_file(file_name=STP_FILES_FOLDER / file_name)
spatial_rep.refit_geometry(file_name=STP_FILES_FOLDER / file_name)
# spatial_rep.plot()


# Create Components
# from caddee.caddee_core.system_representation.component.component import LiftingSurface, Component
wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Wing,']).keys())
wing = LiftingSurface(name='wing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)

tail_primitive_names = list(spatial_rep.get_primitives(search_names=['HT']).keys())
horizontal_stabilizer = LiftingSurface(name='tail', spatial_representation=spatial_rep, primitive_names=tail_primitive_names)

fuselage_primitive_names = list(spatial_rep.get_primitives(search_names=['Fuselage']))
fuselage = Component(name='fuselage', spatial_representation=spatial_rep, primitive_names=fuselage_primitive_names)

sys_rep.add_component(wing)
sys_rep.add_component(horizontal_stabilizer)
sys_rep.add_component(fuselage)

# wing.plot()
# horizontal_stabilizer.plot()
# fuselage.plot()

# Meshes definitions
# Wing VLM Mesh
num_spanwise_vlm = 22
num_chordwise_vlm = 5
leading_edge = wing.project(am.linspace(am.array([7.5, -13.5, 2.5]), am.array([7.5, 13.5, 2.5]), num_spanwise_vlm),
                            direction=am.array([0., 0., -1.]))  # returns MappedArray
trailing_edge = wing.project(np.linspace(np.array([13., -13.5, 2.5]), np.array([13., 13.5, 2.5]), num_spanwise_vlm),
                             direction=np.array([0., 0., -1.]))   # returns MappedArray
chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
wing_upper_surface_wireframe = wing.project(chord_surface.value + np.array([0., 0., 1.5]), direction=np.array([0., 0., -1.]), grid_search_n=100)
wing_lower_surface_wireframe = wing.project(chord_surface.value - np.array([0., 0., 1.5]), direction=np.array([0., 0., 1.]), grid_search_n=100)
wing_camber_surface = am.linspace(wing_upper_surface_wireframe, wing_lower_surface_wireframe, 1) # this linspace will return average when n=1
wing_camber_surface = wing_camber_surface.reshape((num_chordwise_vlm, num_spanwise_vlm, 3))
sys_rep.add_output(name='chord_distribution', quantity=am.norm(leading_edge-trailing_edge))

# Wing AFrame Mesh
num_beam = 35  # num ribs of pegasus
leading_edge = wing.project(np.linspace(np.array([7.5, -13.5, 2.5]), np.array([7.5, 13.5, 2.5]), num_beam), direction=np.array([0., 0., -1.]), plot=False)
trailing_edge = wing.project(np.linspace(np.array([13., -13.5, 2.5]), np.array([13., 13.5, 2.5]), num_beam), direction=np.array([0., 0., -1.]), plot=False)
wing_beam_mesh = am.linear_combination(leading_edge,trailing_edge,1,start_weights=np.ones((num_beam,))*0.67,stop_weights=np.ones((num_beam,))*0.33)
wing_width = am.norm((leading_edge - trailing_edge)*0.43)

wing_top = wing.project(wing_beam_mesh+np.array([0., 0., 1.]), direction=np.array([0., 0., -1.]), plot=False)
wing_bot = wing.project(wing_beam_mesh-np.array([0., 0., 1.]), direction=np.array([0., 0., 1.]), plot=False)
wing_height = am.norm((wing_top - wing_bot)*0.75)
#print('left_width', left_width)
#print('left_height', left_height)
# spatial_rep.plot_meshes([wing_beam_mesh])

# pass the beam meshes to aframe:
beam_mesh = LinearBeamMesh(
    meshes=dict(
    wing_beam_mesh=wing_beam_mesh.reshape((-1,3)),
    wing_beam_width=wing_width,
    wing_beam_height=wing_height,
    ))

# # create the beam dictionaries:
# beams, bounds, joints = {}, {}, {}
# beams['left_wing_beam'] = {'E': 69E9,'G': 26E9,'rho': 2700,'type': 'box','n': num_left_beam}
# beams['right_wing_beam'] = {'E': 69E9,'G': 26E9,'rho': 2700,'type': 'box','n': num_right_beam}

# # the boundary conditions:
# bounds['bc1'] = {'beam': 'left_wing_beam','fpos': 'b','fdim': [1,1,1,1,1,1]}
# bounds['bc2'] = {'beam': 'right_wing_beam','fpos': 'a','fdim': [1,1,1,1,1,1]}

# create the beam dictionaries:
beams, bounds, joints = {}, {}, {}
beams['wing_beam'] = {'E': 69E9,'G': 26E9,'rho': 2700,'cs': 'box','nodes': list(range(num_beam))}
 

# the boundary conditions:
bounds['bc1'] = {'beam': 'wing_beam','node': int(num_beam/2),'fdim': [1,1,1,1,1,1]}


# create the beam model:
beam = LinearBeam(component=wing, mesh=beam_mesh, beams=beams, bounds=bounds, joints=joints)

beam.set_module_input('wing_beamt_cap_in', val=0.01, dv_flag=True, lower=0.0001, scaler=1E3)

beam.set_module_input('wing_beamt_web_in', val=0.01, dv_flag=True, lower=0.0001, scaler=1E3)



sys_rep.add_output('wing_camber_surface', wing_camber_surface)
sys_rep.add_output('wing_beam_mesh', wing_beam_mesh)



#sys_rep_model = sys_rep.assemble_csdl()
#my_model = csdl.Model()
#my_model.add(sys_rep_model, 'sys_rep')

# the system model:
caddee.system_model = system_model = cd.SystemModel()

# design scenario
dc = cd.DesignScenario(name='recon_mission')
dc.equations_of_motion_csdl = cd.EulerFlatEarth6DoFGenRef

# aircraft condition 
ha_cruise = cd.AircraftCondition(name='high_altitude_cruise',stability_flag=False,dynamic_flag=False,)

ha_cruise.atmosphere_model = cd.SimpleAtmosphereModel()
ha_cruise.set_module_input('mach_number', 0.17, dv_flag=True, lower=0.1, upper=0.3, scaler=1)
ha_cruise.set_module_input('time', 3600)
ha_cruise.set_module_input('roll_angle', 0)
ha_cruise.set_module_input('pitch_angle', np.deg2rad(0))
ha_cruise.set_module_input('yaw_angle', 0)
ha_cruise.set_module_input('flight_path_angle', np.deg2rad(0))
ha_cruise.set_module_input('wind_angle', 0)
ha_cruise.set_module_input('observer_location', np.array([0, 0, 1000]))
ha_cruise.set_module_input('altitude', 15240)

# # UFL is unified form language
# # csdl: computational system design language (Unified Computational System Design Language)
# # idea M3L: unified multifidelity multidisciplinary modeling language

cruise_wing_structural_nodal_displacements_mesh = am.vstack((wing_upper_surface_wireframe, wing_lower_surface_wireframe))
# spatial_rep.plot_meshes([cruise_wing_structural_nodal_displacements_mesh], mesh_plot_types=['mesh', 'point_cloud'])
cruise_wing_aero_nodal_displacements_mesh = cruise_wing_structural_nodal_displacements_mesh
cruise_wing_structural_nodal_force_mesh = cruise_wing_structural_nodal_displacements_mesh
cruise_wing_aero_nodal_force_mesh = cruise_wing_structural_nodal_displacements_mesh

order_u = 3
num_control_points_u = 35
knots_u_beginning = np.zeros((order_u-1,))
knots_u_middle = np.linspace(0., 1., num_control_points_u+2)
knots_u_end = np.ones((order_u-1,))
knots_u = np.hstack((knots_u_beginning, knots_u_middle, knots_u_end))
order_v = 1
knots_v = np.array([0., 0.5, 1.])

dummy_b_spline_space = lg.BSplineSpace(name='dummy_b_spline_space', order=(order_u,1), knots=(knots_u,knots_v))
dummy_function_space = lg.BSplineSetSpace(name='dummy_space', b_spline_spaces={'dummy_b_spline_space': dummy_b_spline_space})

cruise_wing_pressure = m3l.Function(name='cruise_wing_pressure', function_space=dummy_function_space)
cruise_wing_displacement = m3l.Function(name='cruise_wing_displacement', function_space=dummy_function_space)

cruise_structural_wing_nodal_forces = cruise_wing_pressure(mesh=cruise_wing_structural_nodal_force_mesh)
cruise_structural_wing_nodal_displacements, cruise_structural_wing_nodal_rotations = beam.evaluate(
    nodal_outputs_mesh=cruise_wing_structural_nodal_displacements_mesh,
    nodal_force=cruise_structural_wing_nodal_forces)
cruise_wing_displacement_output = cruise_wing_displacement.inverse_evaluate(cruise_structural_wing_nodal_displacements)


aerostructural_coupling = m3l.ModelGroup()
# aerostructural_coupling.add_output(cruise_wing_displacement_output)

testing_csdl_model = aerostructural_coupling._assemble_csdl()
testing_csdl_model.create_input('wing_beam_mesh', wing_beam_mesh.value.reshape((-1,3)))
testing_csdl_model.create_input('wing_beam_width', wing_width.value)
testing_csdl_model.create_input('wing_beam_height', wing_height.value)
force_vector = np.zeros((num_beam,3))
force_vector[:,2] = 50000
cruise_wing_forces = testing_csdl_model.create_input('cruise_wing_pressure_input', val=force_vector)
testing_csdl_model.connect('cruise_wing_pressure_input', 'cruise_wing_pressure')

# ''' Approach where user works with CSDL directly '''
# # In short-medium term, want to use the geometry object as it is implemented.
# # -- Want to be able to efficiently and accurately evaluate functions over meshes.
# #   -- Challenge 1: The function spaces for the states do not exist when the meshes are created.
# #       -- Solution 1: Create the function spaces for states before any meshes are made, output states like material properties.
# #           -- Note: Doesn't really fit the API/philosophy we have set out, but might work best.
# #           -- NOTE: After pursuing this, it seems like the wrong approach. It doesn't fit the philosophy, and it's still not clear
# #               how to return a function space object that can be evaluated at a mesh. This approach would be about directly computing a mesh
# #               of state values. It also doesn't seem possible to perform integration for intrinsic->extrinsic with this method in the evaluate.
# #       -- Solution 2: Perform projections when mesh is passed into evaluate call.
# #           -- Note: Repeated projections (computational expense), if not on geometry (like camber), then can get really unintuitive results
# #       -- Solution 3: Save parametric coordinates and primitive name in Mesh/MappedArray.
# #           -- Note: Must create new object, also instantly unclear what to do with coordinates once an interpolation is defined.

# # wing_geometry_function_space = lg.BSplineSetSpace(name='wing_geometry_function_space', b_spline_spaces=wing.get_geometry_primitives())
# traction_function_space = lg.BSplineSetSpace(name='wing_geometry_function_space', b_spline_spaces=wing.get_geometry_primitives())
# displacement_function_space = lg.BSplineSetSpace(name='wing_geometry_function_space', b_spline_spaces=wing.get_geometry_primitives())

# cruise_csdl = csdl.Model()
# traction_coefficients = cruise_csdl.declare_variable('traction_coefficients', shape=(traction_function_space.num_control_points,3))

# nodal_forces = traction_function_space.evaluate(coefficients=traction_coefficients, mesh=cruise_wing_structural_nodal_force_mesh)
# nodal_displacements, nodal_rotations = beam.evaluate(
#     nodal_outputs_mesh=cruise_wing_structural_nodal_displacements_mesh,
#     nodal_force=nodal_forces)
# displacement_coefficients = displacement_function_space.inverse_evaluate(nodal_displacements)

# cruise_csdl.register_output('displacement_coefficients', displacement_coefficients)
# # ha_cruise.add_model(cruise_csdl)

# cruise_csdl.create_input('wing_beam_mesh', wing_beam_mesh.value.reshape((-1,3)))
# cruise_csdl.create_input('wing_beam_width', wing_width.value)
# cruise_csdl.create_input('wing_beam_height', wing_height.value)
# force_vector = np.zeros((num_beam,3))
# force_vector[:,2] = 50000
# cruise_wing_forces = cruise_csdl.create_input('traction_coefficients_input', val=force_vector)
# cruise_csdl.connect('traction_coefficients_input', 'traction_coefficients')

dummy_b_spline_space = lg.BSplineSpace(name='dummy_b_spline_space', order=(order_u,1), knots=(knots_u,knots_v))
dummy_function_space = lg.BSplineSetSpace(name='dummy_space', b_spline_spaces={'dummy_b_spline_space': dummy_b_spline_space})

cruise_wing_pressure = m3l.Function(name='cruise_wing_pressure', function_space=dummy_function_space)
cruise_wing_displacement = m3l.Function(name='cruise_wing_displacement', function_space=dummy_function_space)

cruise_structural_wing_nodal_forces = cruise_wing_pressure(mesh=cruise_wing_structural_nodal_force_mesh)
cruise_structural_wing_nodal_displacements, cruise_structural_wing_nodal_rotations = beam.evaluate(
    nodal_outputs_mesh=cruise_wing_structural_nodal_displacements_mesh,
    nodal_force=cruise_structural_wing_nodal_forces)
cruise_wing_displacement_output = cruise_wing_displacement.inverse_evaluate(cruise_structural_wing_nodal_displacements)


aerostructural_coupling = m3l.ModelGroup()
aerostructural_coupling.register_output(cruise_wing_displacement_output)

testing_csdl_model = aerostructural_coupling._assemble_csdl()
testing_csdl_model.create_input('wing_beam_mesh', wing_beam_mesh.value.reshape((-1,3)))
testing_csdl_model.create_input('wing_beam_width', wing_width.value)
testing_csdl_model.create_input('wing_beam_height', wing_height.value)
force_vector = np.zeros((num_beam,3))
force_vector[:,2] = 50000
cruise_wing_forces = testing_csdl_model.create_input('cruise_wing_pressure_input', val=force_vector)
testing_csdl_model.connect('cruise_wing_pressure_input', 'cruise_wing_pressure')


sim = Simulator(testing_csdl_model, analytics=True)
sim.run()

import vedo
plotter = vedo.Plotter()
points = vedo.Points(sim['wing_beam_displacement'] + wing_beam_mesh.value.reshape((-1,3)))
plotter.show([points], interactive=True, axes=1)    # Plotting beam solution

plotter = vedo.Plotter()
plotting_point_cloud = cruise_wing_structural_nodal_displacements_mesh.value.reshape((-1,3)) + sim['beam_nodal_displacement_wing']
points = vedo.Points(plotting_point_cloud)
plotter.show([points], interactive=True, axes=1)    # Plotting point cloud
spatial_rep.plot_meshes([plotting_point_cloud.reshape(cruise_wing_structural_nodal_displacements_mesh.shape)],
                        mesh_plot_types=['mesh'], primitives=['none'])  # Plotting "framework" solution (using vedo for fitting)


# TODO:
# 1. Add Nick's force map
# 2. Remove any/all hardcoding
# 3. Create helper funtions to make solver API easier
# 3.5. Create docstrings/method headers
# 4. Create solution to not knowing the output space of the function
# 5. Implement real methods for evaluate and inverse_evaluate (may need to add a step before for implementing all of geometry :/)

# sim.compute_total_derivatives()
# sim.check_totals()

# # Option B
# cruise_wing_structural_nodal_displacements_mesh = am.vstack((wing_upper_surface_wireframe, wing_lower_surface_wireframe))
# cruise_wing_aero_nodal_displacements_mesh = cruise_wing_structural_nodal_displacements_mesh
# cruise_wing_structural_nodal_force_mesh = cruise_wing_structural_nodal_displacements_mesh
# cruise_wing_aero_nodal_force_mesh = cruise_wing_structural_nodal_displacements_mesh

# # Declare framework state functions 
# cruise_wing_pressure = m3l.State(name='cruise_wing_pressure', shape=(34,3), region=list(wing.get_geometry_primitives().values()))
# cruise_wing_displacement = m3l.State(name='cruise_wing_displacement', shape=(50,3), region=list(wing.get_geometry_primitives().values()))

# # Use information from framework state functions to compute model solution

# # Set new value for framework state function

# # Define coupling as group?


# # Option C




ha_cruise.mechanics_group = mech_group = cd.MechanicsGroup()
# mech_group.add_module(beam)
mech_group.add_module(aerostructural_coupling, model_group=True)

dc.add_design_condition(design_condition=ha_cruise)
system_model.add_design_scenario(design_scenario=dc)
caddee_csdl = caddee.assemble_csdl_modules()

caddee_csdl.create_input('left_forces', val=left_forces)
caddee_csdl.connect('left_forces', 'system_model.recon_mission.mechanics_group.wing_linear_beam.BeamGroup.GlobalLoads.left_wing_beam_forces')
caddee_csdl.create_input('right_forces', val=right_forces)
caddee_csdl.connect('right_forces', 'system_model.recon_mission.mechanics_group.wing_linear_beam.BeamGroup.GlobalLoads.right_wing_beam_forces')

caddee_csdl.create_input('left_moments', val=left_moments)
caddee_csdl.connect('left_moments', 'system_model.recon_mission.mechanics_group.wing_linear_beam.BeamGroup.GlobalLoads.left_wing_beam_moments')
caddee_csdl.create_input('right_moments', val=right_moments)
caddee_csdl.connect('right_moments', 'system_model.recon_mission.mechanics_group.wing_linear_beam.BeamGroup.GlobalLoads.right_wing_beam_moments')


# beam stress constraints:
#caddee_csdl.add_constraint('system_model.recon_mission.mechanics_group.left_wing_linear_beam.BeamGroup.max_stress',upper=500E6/1.5,scaler=1E-8) # 333.33 MPa
caddee_csdl.add_constraint('system_model.recon_mission.mechanics_group.wing_linear_beam.BeamGroup.vonmises_stress',upper=500E6/1,scaler=1E-8)

caddee_csdl.add_objective('system_model.recon_mission.total_mass_properties.m_total', scaler=1e-3)


sim = Simulator(caddee_csdl, analytics=True)
sim.run()
sim.compute_total_derivatives()
sim.check_totals()


# prob = CSDLProblem(problem_name='pegasus', simulator=sim)
# optimizer = SLSQP(prob, maxiter=1000, ftol=1E-6)
# optimizer.solve()
# optimizer.print_results()


