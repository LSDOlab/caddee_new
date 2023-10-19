'''Example 2 : Description of example 2'''
import caddee.api as cd
from caddee import GEOMETRY_FILES_FOLDER
import csdl
from python_csdl_backend import Simulator
from aframe.core.beam_module import LinearBeamMesh
import aframe.core.beam_module as ebbeam
from modopt.scipy_library import SLSQP
from modopt.csdl_library import CSDLProblem
import csdl
import numpy as np
import array_mapper as am
import m3l
import lsdo_geo as lg
from caddee.core.caddee_core.system_representation.component.component import LiftingSurface, Component


caddee = cd.CADDEE()
caddee.system_representation = sys_rep = cd.SystemRepresentation()
caddee.system_parameterization = sys_param = cd.SystemParameterization(system_representation=sys_rep)

file_name = 'pegasus.stp'

spatial_rep = sys_rep.spatial_representation
spatial_rep.import_file(file_name=GEOMETRY_FILES_FOLDER / file_name)
spatial_rep.refit_geometry(file_name=GEOMETRY_FILES_FOLDER / file_name)

# Create Components
wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Wing,']).keys())
wing = LiftingSurface(name='wing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)

tail_primitive_names = list(spatial_rep.get_primitives(search_names=['HT']).keys())
horizontal_stabilizer = LiftingSurface(name='tail', spatial_representation=spatial_rep, primitive_names=tail_primitive_names)

fuselage_primitive_names = list(spatial_rep.get_primitives(search_names=['Fuselage']))
fuselage = Component(name='fuselage', spatial_representation=spatial_rep, primitive_names=fuselage_primitive_names)

sys_rep.add_component(wing)
sys_rep.add_component(horizontal_stabilizer)
sys_rep.add_component(fuselage)

# Meshes definitions
# Wing VLM Mesh
num_spanwise_vlm = 22
num_chordwise_vlm = 5
leading_edge = wing.project(am.linspace(am.array([7.5, -13.5, 2.5]), am.array([7.5, 13.5, 2.5]), num_spanwise_vlm),
                            direction=am.array([0., 0., -1.]))  # returns MappedArray
trailing_edge = wing.project(np.linspace(np.array([13., -13.5, 2.5]), np.array([13., 13.5, 2.5]), num_spanwise_vlm),
                             direction=np.array([0., 0., -1.]))   # returns MappedArray
chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
wing_upper_surface_wireframe = wing.project(chord_surface.value + np.array([0., 0., 1.5]), direction=np.array([0., 0., -1.]), grid_search_n=25)
wing_lower_surface_wireframe = wing.project(chord_surface.value - np.array([0., 0., 1.5]), direction=np.array([0., 0., 1.]), grid_search_n=25)
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

# # create the beam model:  # OLD Beam model definition. See below for M3L definition and use.
# beam = LinearBeam(component=wing, mesh=beam_mesh, beams=beams, bounds=bounds, joints=joints)
# beam.set_module_input('wing_beamt_cap_in', val=0.01, dv_flag=True, lower=0.0001, scaler=1E3)
# beam.set_module_input('wing_beamt_web_in', val=0.01, dv_flag=True, lower=0.0001, scaler=1E3)

sys_rep.add_output('wing_camber_surface', wing_camber_surface)
sys_rep.add_output('wing_beam_mesh', wing_beam_mesh)

# the system model:
caddee.system_model = system_model = cd.SystemModel()

# design scenario
dc = cd.DesignScenario(name='recon_mission')

# # aircraft condition 
# cruise_condition = cd.CruiseCondition(name="cruise_1")
# cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()
# cruise_condition.set_module_input(name='altitude', val=1000)
# cruise_condition.set_module_input(name='mach_number', val=0.17, dv_flag=True, lower=0.17, upper=0.19)
# cruise_condition.set_module_input(name='range', val=40000)
# cruise_condition.set_module_input(name='pitch_angle', val=np.deg2rad(0), dv_flag=True, lower=0., upper=np.deg2rad(10))
# cruise_condition.set_module_input(name='flight_path_angle', val=0)
# cruise_condition.set_module_input(name='roll_angle', val=0)
# cruise_condition.set_module_input(name='yaw_angle', val=0)
# cruise_condition.set_module_input(name='wind_angle', val=0)
# cruise_condition.set_module_input(name='observer_location', val=np.array([0, 0, 500]))

# ac_states = cruise_condition.evaluate_ac_states()
# # cruise_model.register_output(ac_states)

cruise_wing_structural_nodal_displacements_mesh = am.vstack((wing_upper_surface_wireframe, wing_lower_surface_wireframe))
cruise_wing_aero_nodal_displacements_mesh = cruise_wing_structural_nodal_displacements_mesh
cruise_wing_structural_nodal_force_mesh = cruise_wing_structural_nodal_displacements_mesh
cruise_wing_aero_nodal_force_mesh = cruise_wing_structural_nodal_displacements_mesh

order_u = 3
num_control_points_u = 35

dummy_b_spline_space = lg.BSplineSpace(name='dummy_b_spline_space', order=(order_u,1), control_points_shape=(num_control_points_u,1))
dummy_function_space = lg.BSplineSetSpace(name='dummy_space', spaces={'dummy_b_spline_space': dummy_b_spline_space})

cruise_wing_pressure_coefficients = m3l.Variable(name='cruise_wing_pressure_coefficients', shape=(num_control_points_u,1,3),
                                                 value=np.zeros((num_control_points_u,1,3)))
cruise_wing_pressure = m3l.Function(name='cruise_wing_pressure', space=dummy_function_space, coefficients=cruise_wing_pressure_coefficients)

cruise_wing_displacement_coefficients = m3l.Variable(name='cruise_wing_displacement_coefficients', shape=(num_control_points_u,1,3),
                                                     value=np.zeros((num_control_points_u,1,3)))
cruise_wing_displacement = m3l.Function(name='cruise_wing_displacement', space=dummy_function_space, coefficients=cruise_wing_displacement_coefficients)

### Start defining computational graph ###

cruise_structural_wing_nodal_forces = cruise_wing_pressure(mesh=cruise_wing_structural_nodal_force_mesh)

beam_force_map_model = ebbeam.EBBeamForces(component=wing, beams=beams, beam_mesh=beam_mesh)
cruise_structural_wing_mesh_forces = beam_force_map_model.evaluate(nodal_forces=cruise_structural_wing_nodal_forces,
                                                                   nodal_forces_mesh=cruise_wing_structural_nodal_force_mesh)

beam_displacements_model = ebbeam.EBBeam(component=wing, mesh=beam_mesh, beams=beams, bounds=bounds, joints=joints)
beam_displacements_model.set_module_input('wing_beamt_cap_in', val=0.01, dv_flag=True, lower=0.0001, scaler=1E3)
beam_displacements_model.set_module_input('wing_beamt_web_in', val=0.01, dv_flag=True, lower=0.0001, scaler=1E3)

cruise_structural_wing_mesh_displacements, cruise_structural_wing_mesh_rotations, _, _, _ = beam_displacements_model.evaluate(
    forces=cruise_structural_wing_mesh_forces)

beam_displacement_map_model = ebbeam.EBBeamNodalDisplacements(component=wing, beams=beams, beam_mesh=beam_mesh)
cruise_structural_wing_nodal_displacements = beam_displacement_map_model.evaluate(beam_displacements=cruise_structural_wing_mesh_displacements,
                                                                        nodal_displacements_mesh=cruise_wing_structural_nodal_displacements_mesh)

test = cruise_structural_wing_nodal_displacements + cruise_structural_wing_nodal_forces

cruise_model = m3l.Model()
cruise_model.register_output(cruise_structural_wing_nodal_displacements)
cruise_model.register_output(test)

testing_csdl_model = cruise_model.assemble_csdl()
testing_csdl_model.create_input('wing_beam_mesh', wing_beam_mesh.value.reshape((-1,3)))
testing_csdl_model.create_input('wing_beam_width', wing_width.value)
testing_csdl_model.create_input('wing_beam_height', wing_height.value)
force_vector = np.zeros((num_beam,3))
force_vector[:,2] = 3000
cruise_wing_forces = testing_csdl_model.create_input('cruise_wing_pressure_input', val=force_vector)
testing_csdl_model.connect('cruise_wing_pressure_input', 'cruise_wing_pressure_evaluation.'+cruise_wing_pressure_coefficients.name) 


sim = Simulator(testing_csdl_model, analytics=True)
sim.run()

import vedo
plotter = vedo.Plotter()
points = vedo.Points(sim['wing_eb_beam_model.wing_beam_displacement'] + wing_beam_mesh.value.reshape((-1,3)))
plotter.show([points], interactive=True, axes=1)    # Plotting beam solution

plotter = vedo.Plotter()
plotting_point_cloud = cruise_wing_structural_nodal_displacements_mesh.value.reshape((-1,3)) + sim['wing_eb_beam_displacement_map.wing_beam_nodal_displacement']
points = vedo.Points(plotting_point_cloud)
plotter.show([points], interactive=True, axes=1)    # Plotting point cloud
spatial_rep.plot_meshes([plotting_point_cloud.reshape(cruise_wing_structural_nodal_displacements_mesh.shape)],
                        mesh_plot_types=['mesh'], primitives=['none'])  # Plotting "framework" solution (using vedo for fitting)

# sim.compute_total_derivatives()
# sim.check_totals()



exit()

# eom_m3l_model = cd.EoMEuler6DOF()
# trim_residual = eom_m3l_model.evaluate(
#     total_mass=total_mass, 
#     total_cg_vector=total_cg, 
#     total_inertia_tensor=total_inertia, 
#     total_forces=total_forces, 
#     total_moments=total_moments,
#     ac_states=ac_states
# )

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



