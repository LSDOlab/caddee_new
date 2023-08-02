import caddee.api as cd
from python_csdl_backend import Simulator
import numpy as np
import array_mapper as am
from aframe.core.beam_module import EBBeam, LinearBeamMesh
from aframe.core.mass import Mass, MassMesh
from VAST.core.vast_solver import VASTFluidSover
from VAST.core.fluid_problem import FluidProblem
from VAST.core.generate_mappings_m3l import VASTNodalForces
# from modopt.snopt_library import SNOPT
from modopt.scipy_library import SLSQP
from modopt.csdl_library import CSDLProblem
import csdl
from caddee.core.caddee_core.system_representation.component.component import LiftingSurface, Component
from caddee import GEOMETRY_FILES_FOLDER
import m3l
import lsdo_geo as lg
import aframe.core.beam_module as ebbeam

caddee = cd.CADDEE()
caddee.system_model = system_model = cd.SystemModel()
caddee.system_representation = sys_rep = cd.SystemRepresentation()
caddee.system_parameterization = sys_param = cd.SystemParameterization(system_representation=sys_rep)

file_name = 'AmazonPrime.stp'

spatial_rep = sys_rep.spatial_representation
spatial_rep.import_file(file_name=file_name)
spatial_rep.refit_geometry(file_name=GEOMETRY_FILES_FOLDER / file_name)
#spatial_rep.plot()


# Wing definintions - Mid, Upper, Lower Wings, then Top, Bottom of frame, finally vertical stabilizer
wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Horizontal Wing']).keys())
MidWing = LiftingSurface(name='MidWing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names) 

wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Upper Wing']).keys())
TopWing = LiftingSurface(name='TopWing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)

wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Lower Wing']).keys())
BotWing = LiftingSurface(name='BotWing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)

wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Top of Frame']).keys())
TopFrame = LiftingSurface(name='TopFrame', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)

wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Lower Frame']).keys())
BotFrame = LiftingSurface(name='BotFrame', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)

wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Vertical Stabilizer']).keys())
VertStab = LiftingSurface(name='VertStab', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)

# wing plots - check progress
MidWing.plot()
TopWing.plot()
BotWing.plot()
TopFrame.plot()
BotFrame.plot()
VertStab.plot()



# region Rotors
# Pusher prop
pp_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Middle Props, 0']).keys())
ppm_left = cd.Rotor(name='ppm_disk_left', spatial_representation=spatial_rep, primitive_names=pp_disk_prim_names)
sys_rep.add_component(ppm_left)
# ppm_left.plot()

pp_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Middle Props, 1']).keys())
ppm_right = cd.Rotor(name='ppm_disk_right', spatial_representation=spatial_rep, primitive_names=pp_disk_prim_names)
sys_rep.add_component(ppm_right)
# ppm_right.plot()

pp_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Upper Props, 1']).keys())
ppu_left = cd.Rotor(name='ppu_disk_left', spatial_representation=spatial_rep, primitive_names=pp_disk_prim_names)
sys_rep.add_component(ppu_left)
# ppu_left.plot()

pp_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Upper Props, 0']).keys())
ppu_right = cd.Rotor(name='ppu_disk_right', spatial_representation=spatial_rep, primitive_names=pp_disk_prim_names)
sys_rep.add_component(ppu_right)
# ppu_right.plot()

pp_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Lower Props, 1']).keys())
ppl_left = cd.Rotor(name='ppl_disk_left', spatial_representation=spatial_rep, primitive_names=pp_disk_prim_names)
sys_rep.add_component(ppl_left)
# ppl_left.plot()

pp_disk_prim_names = list(spatial_rep.get_primitives(search_names=['Lower Props, 0']).keys())
ppl_right = cd.Rotor(name='ppl_disk_right', spatial_representation=spatial_rep, primitive_names=pp_disk_prim_names)
sys_rep.add_component(ppl_right)
# ppl_right.plot()

whole_geometry_component_primitive_names = list(spatial_rep.get_primitives().keys())
whole_geometry_component = cd.Component(name='whole_geometry', spatial_representation=spatial_rep, 
                                        primitive_names=whole_geometry_component_primitive_names)

# endregion
# Rotor plots - check progress
#ppm.plot()
#ppu.plot()
#ppl.plot()


# Adding components
sys_rep.add_component(MidWing)
sys_rep.add_component(TopWing)
sys_rep.add_component(BotWing)
sys_rep.add_component(TopFrame)
sys_rep.add_component(BotFrame)
sys_rep.add_component(VertStab)

do_plots=False

# region FFD
ppm_left_geometry_primitives = ppm_left.get_geometry_primitives()
ppm_left_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(
    ppm_left_geometry_primitives,
    num_control_points=(2, 2, 2), order=(2,2,2),
    xyz_to_uvw_indices=(0,1,2)
)
ppm_left_ffd_block = cd.SRBGFFDBlock(name='ppm_left_ffd_block',
                                  primitive=ppm_left_ffd_bspline_volume,
                                  embedded_entities=ppm_left_geometry_primitives)
ppm_left_ffd_block.add_scale_v(name='ppm_left_scale_v',order=1, num_dof=1, cost_factor=1.)
ppm_left_ffd_block.add_scale_w(name='ppm_left_scale_w', order=1, num_dof=1)
# ppm_left_ffd_block.plot()

ppm_right_geometry_primitives = ppm_right.get_geometry_primitives()
ppm_right_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(
    ppm_right_geometry_primitives,
    num_control_points=(2, 2, 2), order=(2,2,2),
    xyz_to_uvw_indices=(0,1,2)
)
ppm_right_ffd_block = cd.SRBGFFDBlock(name='ppm_right_ffd_block',
                                  primitive=ppm_right_ffd_bspline_volume,
                                  embedded_entities=ppm_right_geometry_primitives)
ppm_right_ffd_block.add_scale_v(name='ppm_right_scale_v',order=1, num_dof=1, cost_factor=1.)
ppm_right_ffd_block.add_scale_w(name='ppm_right_scale_w', order=1, num_dof=1)

ppu_left_geometry_primitives = ppu_left.get_geometry_primitives()
ppu_left_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(
    ppu_left_geometry_primitives,
    num_control_points=(2, 2, 2), order=(2,2,2),
    xyz_to_uvw_indices=(0,1,2)
)
ppu_left_ffd_block = cd.SRBGFFDBlock(name='ppu_left_ffd_block',
                                  primitive=ppu_left_ffd_bspline_volume,
                                  embedded_entities=ppu_left_geometry_primitives)
ppu_left_ffd_block.add_scale_v(name='ppu_left_scale_v',order=1, num_dof=1, cost_factor=1.)
ppu_left_ffd_block.add_scale_w(name='ppu_left_scale_w', order=1, num_dof=1)
ppu_left_ffd_block.add_translation_u(name='ppu_left_translation_u', order=2, num_dof=2, cost_factor=1.5)
# ppu_left_ffd_block.plot()

ppu_right_geometry_primitives = ppu_right.get_geometry_primitives()
ppu_right_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(
    ppu_right_geometry_primitives,
    num_control_points=(2, 2, 2), order=(2,2,2),
    xyz_to_uvw_indices=(0,1,2)
)
ppu_right_ffd_block = cd.SRBGFFDBlock(name='ppu_right_ffd_block',
                                  primitive=ppu_right_ffd_bspline_volume,
                                  embedded_entities=ppu_right_geometry_primitives)
ppu_right_ffd_block.add_scale_v(name='ppu_right_scale_v',order=1, num_dof=1, cost_factor=1.)
ppu_right_ffd_block.add_scale_w(name='ppu_right_scale_w', order=1, num_dof=1)
ppu_right_ffd_block.add_translation_u(name='ppu_right_translation_u', order=2, num_dof=2, cost_factor=1.5)

ppl_left_geometry_primitives = ppl_left.get_geometry_primitives()
ppl_left_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(
    ppl_left_geometry_primitives,
    num_control_points=(2, 2, 2), order=(2,2,2),
    xyz_to_uvw_indices=(0,1,2)
)
ppl_left_ffd_block = cd.SRBGFFDBlock(name='ppl_left_ffd_block',
                                  primitive=ppl_left_ffd_bspline_volume,
                                  embedded_entities=ppl_left_geometry_primitives)
ppl_left_ffd_block.add_scale_v(name='ppl_left_scale_v',order=1, num_dof=1, cost_factor=1.)
ppl_left_ffd_block.add_scale_w(name='ppl_left_scale_w', order=1, num_dof=1)
ppl_left_ffd_block.add_translation_u(name='ppl_left_translation_u', order=2, num_dof=2, cost_factor=1.5)


ppl_right_geometry_primitives = ppl_right.get_geometry_primitives()
ppl_right_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(
    ppl_right_geometry_primitives,
    num_control_points=(2, 2, 2), order=(2,2,2),
    xyz_to_uvw_indices=(0,1,2)
)
ppl_right_ffd_block = cd.SRBGFFDBlock(name='ppl_right_ffd_block',
                                  primitive=ppl_right_ffd_bspline_volume,
                                  embedded_entities=ppl_right_geometry_primitives)
ppl_right_ffd_block.add_scale_v(name='ppl_right_scale_v',order=1, num_dof=1, cost_factor=1.)
ppl_right_ffd_block.add_scale_w(name='ppl_right_scale_w', order=1, num_dof=1)
ppl_right_ffd_block.add_translation_u(name='ppl_right_translation_u', order=2, num_dof=2, cost_factor=1.5)

# endregion

# region pusher prop (pp) meshes

# disk: middle 
#left
y11 = ppm_left.project(np.array([2.5,-.93,0]), direction=np.array([-1., 0., 0.]), plot=False)
y12 = ppm_left.project(np.array([2.5,-2.57,0]), direction=np.array([-1., 0., 0.]), plot=False)
y21 = ppm_left.project(np.array([2.5,-1.75,0.82]), direction=np.array([-1., 0., 0.]), plot=False)
y22 = ppm_left.project(np.array([2.5,-1.75,-0.82]), direction=np.array([-1., 0., 0.]), plot=False)
ppm_left_plane_y = am.subtract(y11, y12)
ppm_left_plane_x = am.subtract(y21, y22)
ppm_left_origin = ppm_left.project(np.array([2.5,-1.75,0]), direction=np.array([-1., 0., 0.]))
sys_rep.add_output(f"{ppm_left.parameters['name']}_in_plane_1", ppm_left_plane_y)
sys_rep.add_output(f"{ppm_left.parameters['name']}_in_plane_2", ppm_left_plane_x)
sys_rep.add_output(f"{ppm_left.parameters['name']}_origin", ppm_left_origin)

#right
y11 = ppm_right.project(np.array([2.5,0.93,0]), direction=np.array([-1., 0., 0.]), plot=False)
y12 = ppm_right.project(np.array([2.5,2.57,0]), direction=np.array([-1., 0., 0.]), plot=False)
y21 = ppm_right.project(np.array([2.5,1.75,0.82]), direction=np.array([-1., 0., 0.]), plot=False)
y22 = ppm_right.project(np.array([2.5,1.75,-0.82]), direction=np.array([-1., 0., 0.]), plot=False)
ppm_right_plane_y = am.subtract(y11, y12)
ppm_right_plane_x = am.subtract(y21, y22)
ppm_right_origin = ppm_right.project(np.array([2.5,1.75,0]), direction=np.array([-1., 0., 0.]))
sys_rep.add_output(f"{ppm_right.parameters['name']}_in_plane_1", ppm_right_plane_y)
sys_rep.add_output(f"{ppm_right.parameters['name']}_in_plane_2", ppm_right_plane_x)
sys_rep.add_output(f"{ppm_right.parameters['name']}_origin", ppm_right_origin)

# disk: uppers
#left
y11 = ppu_left.project(np.array([3.976,-1.458,2.193]), direction=np.array([-.683,0.259,0.683]), plot=False)
y12 = ppu_left.project(np.array([3.024,-0.842,1.007]), direction=np.array([-.683,0.259,0.683]), plot=False)
y21 = ppu_left.project(np.array([3.136,-1.88,1.513]), direction=np.array([-.683,0.259,0.683]), plot=False)
y22 = ppu_left.project(np.array([3.846,-0.42,1.687]), direction=np.array([-.683,0.259,0.683]), plot=False)
ppu_left_plane_y = am.subtract(y11, y12)
ppu_left_plane_x = am.subtract(y21, y22)
ppu_left_origin = ppu_left.project(np.array([3.5,-1.15,1.6]), direction=np.array([-.683,0.259,0.683]))
sys_rep.add_output(f"{ppu_left.parameters['name']}_in_plane_1", ppu_left_plane_y)
sys_rep.add_output(f"{ppu_left.parameters['name']}_in_plane_2", ppu_left_plane_x)
sys_rep.add_output(f"{ppu_left.parameters['name']}_origin", ppu_left_origin)

#right
y11 = ppu_right.project(np.array([3.976,1.458,2.193]), direction=np.array([-.683,-0.259,0.683]), plot=False)
y12 = ppu_right.project(np.array([3.024,0.842,1.007]), direction=np.array([-.683,-0.259,0.683]), plot=False)
y21 = ppu_right.project(np.array([3.136,1.88,1.513]), direction=np.array([-.683,-0.259,0.683]), plot=False)
y22 = ppu_right.project(np.array([3.846,0.42,1.687]), direction=np.array([-.683,-0.259,0.683]), plot=False)
ppu_right_plane_y = am.subtract(y11, y12)
ppu_right_plane_x = am.subtract(y21, y22)
ppu_right_origin = ppu_right.project(np.array([3.5,1.15,1.6]), direction=np.array([-.683,-0.259,0.683]))
sys_rep.add_output(f"{ppu_right.parameters['name']}_in_plane_1", ppu_right_plane_y)
sys_rep.add_output(f"{ppu_right.parameters['name']}_in_plane_2", ppu_right_plane_x)
sys_rep.add_output(f"{ppu_right.parameters['name']}_origin", ppu_right_origin)

# disk: lowers
#left
y11 = ppl_left.project(np.array([1.66,-0.775,-1.03]), direction=np.array([-0.75,0.5,0.433]), plot=False)
y12 = ppl_left.project(np.array([0.84,-0.775,-2.45]), direction=np.array([-0.75,0.5,0.433]), plot=False)
y21 = ppl_left.project(np.array([0.895,-0.065,-1.535]), direction=np.array([-0.75,0.5,0.433]), plot=False)
y22 = ppl_left.project(np.array([1.605,-1.485,-1.945]), direction=np.array([-0.75,0.5,0.433]), plot=False)
ppl_left_plane_y = am.subtract(y11, y12)
ppl_left_plane_x = am.subtract(y21, y22)
ppl_left_origin = ppl_left.project(np.array([1.25,0.775,-1.74]), direction=np.array([-0.75,0.5,0.433]))
sys_rep.add_output(f"{ppl_left.parameters['name']}_in_plane_1", ppl_left_plane_y)
sys_rep.add_output(f"{ppl_left.parameters['name']}_in_plane_2", ppl_left_plane_x)
sys_rep.add_output(f"{ppl_left.parameters['name']}_origin", ppl_left_origin)

#right
y11 = ppl_right.project(np.array([1.66,0.775,-1.03]), direction=np.array([-0.75,-0.5,0.433]), plot=False)
y12 = ppl_right.project(np.array([0.84,0.775,-2.45]), direction=np.array([-0.75,-0.5,0.433]), plot=False)
y21 = ppl_right.project(np.array([0.895,0.065,-1.535]), direction=np.array([-0.75,-0.5,0.433]), plot=False)
y22 = ppl_right.project(np.array([1.605,1.485,-1.945]), direction=np.array([-0.75,-0.5,0.433]), plot=False)
ppl_right_plane_y = am.subtract(y11, y12)
ppl_right_plane_x = am.subtract(y21, y22)
ppl_right_origin = ppl_right.project(np.array([1.25,-0.775,-1.74]), direction=np.array([-0.75,-0.5,0.433]))
sys_rep.add_output(f"{ppl_right.parameters['name']}_in_plane_1", ppl_right_plane_y)
sys_rep.add_output(f"{ppl_right.parameters['name']}_in_plane_2", ppl_right_plane_x)
sys_rep.add_output(f"{ppl_right.parameters['name']}_origin", ppl_right_origin)

# endregion

ppm_left_radius_1 = am.norm(ppm_left_plane_x/2)
ppm_left_radius_2 = am.norm(ppm_left_plane_y/2)
sys_param.add_input(name='ppm_left_radius_1', quantity=ppm_left_radius_1, value=np.array([0.5]))
sys_param.add_input(name='ppm_left_radius_2', quantity=ppm_left_radius_2, value=np.array([0.5]))

ppm_right_radius_1 = am.norm(ppm_right_plane_x/2)
ppm_right_radius_2 = am.norm(ppm_right_plane_y/2)
sys_param.add_input(name='ppm_right_radius_1', quantity=ppm_right_radius_1, value=np.array([0.5]))
sys_param.add_input(name='ppm_right_radius_2', quantity=ppm_right_radius_2, value=np.array([0.5]))

ppu_left_radius_1 = am.norm(ppu_left_plane_x/2)
ppu_left_radius_2 = am.norm(ppu_left_plane_y/2)
sys_param.add_input(name='ppu_left_radius_1', quantity=ppu_left_radius_1, value=np.array([0.5]))
sys_param.add_input(name='ppu_left_radius_2', quantity=ppu_left_radius_2, value=np.array([0.5]))

ppu_right_radius_1 = am.norm(ppu_right_plane_x/2)
ppu_right_radius_2 = am.norm(ppu_right_plane_y/2)
sys_param.add_input(name='ppu_right_radius_1', quantity=ppu_right_radius_1, value=np.array([0.5]))
sys_param.add_input(name='ppu_right_radius_2', quantity=ppu_right_radius_2, value=np.array([0.5]))

ppl_left_radius_1 = am.norm(ppl_left_plane_x/2)
ppl_left_radius_2 = am.norm(ppl_left_plane_y/2)
sys_param.add_input(name='ppl_left_radius_1', quantity=ppl_left_radius_1, value=np.array([0.5]))
sys_param.add_input(name='ppl_left_radius_2', quantity=ppl_left_radius_2, value=np.array([0.5]))

ppl_right_radius_1 = am.norm(ppl_right_plane_x/2)
ppl_right_radius_2 = am.norm(ppl_right_plane_y/2)
sys_param.add_input(name='ppl_right_radius_1', quantity=ppl_right_radius_1, value=np.array([0.5]))
sys_param.add_input(name='ppl_right_radius_2', quantity=ppl_right_radius_2, value=np.array([0.5]))

ffd_set = cd.SRBGFFDSet(
    name='ffd_set',
    ffd_blocks={ppm_left_ffd_block.name : ppm_left_ffd_block,
                ppm_right_ffd_block.name : ppm_right_ffd_block,
                ppu_left_ffd_block.name : ppu_left_ffd_block,
                ppu_right_ffd_block.name : ppu_right_ffd_block,
                ppl_left_ffd_block.name : ppl_left_ffd_block,
                ppl_right_ffd_block.name : ppl_right_ffd_block,
                }
)
sys_param.add_geometry_parameterization(ffd_set)
sys_param.setup()

configuration_names = ["hover_configuration", "cruise_configuration"]
system_configurations = sys_rep.declare_configurations(names=configuration_names)
hover_configuration = system_configurations['hover_configuration']
cruise_configuration = system_configurations['cruise_configuration']

actuation_axis_port = whole_geometry_component.project(np.array([3., -1., 0.]), plot=True)
actuation_axis_starboard = whole_geometry_component.project(np.array([3., 1., 0.]), plot=True)
horizontal_stabilizer_actuation_axis = actuation_axis_starboard - actuation_axis_port
from caddee.core.caddee_core.system_representation.prescribed_actuations import PrescribedRotation
hover_actuator_solver = PrescribedRotation(component=whole_geometry_component, axis_origin=actuation_axis_port,
                                                           axis_vector=horizontal_stabilizer_actuation_axis)
hover_actuation_profile = np.linspace(np.array([np.pi/2]))
hover_actuator_solver.set_rotation(name='hover_actuation', value=hover_actuation_profile, units='radians')
hover_configuration.actuate(transformation=hover_actuator_solver)

# TEMPORARY
system_representation_model = sys_rep.assemble_csdl()
system_parameterization_model = sys_param.assemble_csdl()

my_model = csdl.Model()
my_model.add(system_parameterization_model, 'system_parameterization')
my_model.add(system_representation_model, 'system_representation')

sim = Simulator(my_model, analytics=True, display_scripts=True)
sim.run()

hover_geo = sim['hover_configuration_geometry']
spatial_rep.update(hover_geo)
spatial_rep.plot()

exit()
# removed blade meshes, twist

# design scenario
design_scenario = cd.DesignScenario(name='aircraft_hover')

# region hover condition
hover_model = m3l.Model()
hover_condition = cd.CruiseCondition(name="hover_1")
hover_condition.atmosphere_model = cd.SimpleAtmosphereModel()
hover_condition.set_module_input(name='altitude', val=0)
hover_condition.set_module_input(name='mach_number', val=0, dv_flag=True, lower=0, upper=0)
hover_condition.set_module_input(name='range', val=0)
hover_condition.set_module_input(name='pitch_angle', val=np.deg2rad(0), dv_flag=True, lower=0., upper=np.deg2rad(0))
hover_condition.set_module_input(name='flight_path_angle', val=0)
hover_condition.set_module_input(name='roll_angle', val=0)
hover_condition.set_module_input(name='yaw_angle', val=0)
hover_condition.set_module_input(name='wind_angle', val=0)
hover_condition.set_module_input(name='observer_location', val=np.array([0, 0, 0]))

ac_states = hover_condition.evaluate_ac_states()
hover_condition.register_output(ac_states)

# BEM prop forces and moments

# middle
from lsdo_rotor.core.BEM_caddee.BEM_caddee import BEM, BEMMesh
pusher_bem_mesh = BEMMesh(
    meshes=dict(
        ppm_disk_in_plane_1=ppm_plane_y,
        ppm_disk_in_plane_2=ppm_plane_x,
        ppm_disk_origin=ppm_origin,
    ),
    airfoil='NACA_4412',
    num_blades=4,
    num_radial=25,
    use_airfoil_ml=False,
    mesh_units='ft'
)

bem_model_mid = BEM(disk_prefix='ppu', blade_prefix='pp', component=ppu, mesh=pusher_bem_mesh)
bem_model_mid.set_module_input('rpm', val=1350, dv_flag=True, lower=800, upper=2000, scaler=1e-3)
bem_forces, bem_moments, _, _, _ = bem_model_mid.evaluate(ac_states=ac_states)

# create the aframe dictionaries:
joints, bounds, beams = {}, {}, {}
beams['wing_beam'] = {'E': 70E9,'G': 70E9/(2*(1 + 0.33)),'rho': 2700,'cs': 'box','nodes': list(range(num_wing_beam))}
bounds['wing_root'] = {'beam': 'wing_beam','node': 5,'fdim': [1,1,1,1,1,1]}

# create the beam model:
#beam = EBBeam(component=wing, mesh=beam_mesh, beams=beams, bounds=bounds, joints=joints, mesh_units='ft')
#beam_mass = Mass(component=wing, mesh=beam_mass_mesh, beams=beams, mesh_units='ft') # the separate mass model thingy
#beam.set_module_input('wing_beamt_cap_in', val=0.005, dv_flag=True, lower=0.001, upper=0.02, scaler=1E3)
#beam.set_module_input('wing_beamt_web_in', val=0.005, dv_flag=True, lower=0.001, upper=0.02, scaler=1E3)

#cruise_wing_structural_nodal_displacements_mesh = am.vstack((wing_upper_surface_wireframe, wing_lower_surface_wireframe))
#ruise_wing_aero_nodal_displacements_mesh = cruise_wing_structural_nodal_displacements_mesh
#cruise_wing_structural_nodal_force_mesh = cruise_wing_structural_nodal_displacements_mesh
#cruise_wing_aero_nodal_force_mesh = cruise_wing_structural_nodal_displacements_mesh

#dummy_b_spline_space = lg.BSplineSpace(name='dummy_b_spline_space', order=(3,1), control_points_shape=((35,1)))
#dummy_function_space = lg.BSplineSetSpace(name='dummy_space', spaces={'dummy_b_spline_space': dummy_b_spline_space})

#cruise_wing_displacement_coefficients = m3l.Variable(name='cruise_wing_displacement_coefficients', shape=(35,3))
#cruise_wing_displacement = m3l.Function(name='cruise_wing_displacement', space=dummy_function_space, coefficients=cruise_wing_displacement_coefficients)


#beam_force_map_model = ebbeam.EBBeamForces(component=wing, beam_mesh=beam_mesh, beams=beams)
#cruise_structural_wing_mesh_forces = beam_force_map_model.evaluate(nodal_forces=wing_forces,
#                                                                   nodal_forces_mesh=oml_mesh)

#beam_displacements_model = ebbeam.EBBeam(component=wing, mesh=beam_mesh, beams=beams, bounds=bounds, joints=joints)
#beam_displacements_model.set_module_input('wing_beamt_cap_in', val=0.01, dv_flag=True, lower=0.001, upper=0.04, scaler=1E3)
#beam_displacements_model.set_module_input('wing_beamt_web_in', val=0.01, dv_flag=True, lower=0.001, upper=0.04, scaler=1E3)

#cruise_structural_wing_mesh_displacements, cruise_structural_wing_mesh_rotations, wing_mass, wing_cg, wing_inertia_tensor = beam_displacements_model.evaluate(
#    forces=cruise_structural_wing_mesh_forces)

#cruise_model.register_output(cruise_structural_wing_mesh_displacements)


# hmmm I hope this works:
#mass_model_wing_mass = beam_mass.evaluate()

total_mass_properties = cd.TotalMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass_model_wing_mass, battery_mass, mass_m4, wing_cg, cg_battery, cg_m4, wing_inertia_tensor, I_battery, I_m4)
# total_mass, total_cg, total_inertia = total_mass_properties.evaluate(wing_mass, battery_mass, mass_m4, wing_cg, cg_battery, cg_m4, wing_inertia_tensor, I_battery, I_m4)
# total_mass, total_cg, total_inertia = total_mass_properties.evaluate(battery_mass, mass_m4, cg_battery, cg_m4, I_battery, I_m4)

cruise_model.register_output(total_mass)
cruise_model.register_output(total_cg)
cruise_model.register_output(total_inertia)

# inertial forces and moments
inertial_loads_model = cd.InertialLoadsM3L(load_factor=1.)
inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg, totoal_mass=total_mass, ac_states=ac_states)
cruise_model.register_output(inertial_forces)
cruise_model.register_output(inertial_moments)

# total forces and moments 
total_forces_moments_model = cd.TotalForcesMomentsM3L()
total_forces, total_moments = total_forces_moments_model.evaluate(vlm_force, vlm_moment, bem_forces, bem_moments, inertial_forces, inertial_moments)
cruise_model.register_output(total_forces)
cruise_model.register_output(total_moments)

# pass total forces/moments + mass properties into EoM model
eom_m3l_model = cd.EoMM3LEuler6DOF()
trim_residual = eom_m3l_model.evaluate(
    total_mass=total_mass,
    total_cg_vector=total_cg,
    total_inertia_tensor=total_inertia,
    total_forces=total_forces,
    total_moments=total_moments,
    ac_states=ac_states
)

cruise_model.register_output(trim_residual)

# Add cruise m3l model to cruise condition
cruise_condition.add_m3l_model('cruise_model', cruise_model)

# Add design condition to design scenario
design_scenario.add_design_condition(cruise_condition)
# endregion
system_model.add_design_scenario(design_scenario=design_scenario)

caddee_csdl_model = caddee.assemble_csdl()

h_tail_act = caddee_csdl_model.create_input('h_tail_act', val=np.deg2rad(-0.5))
caddee_csdl_model.add_design_variable('h_tail_act',
                                      lower=np.deg2rad(-10),
                                      upper=np.deg2rad(10),
                                      scaler=1,
                                      )


# connections for the new mass model:
caddee_csdl_model.connect('system_model.aircraft_trim.cruise_1.cruise_1.wing_eb_beam_model.wing_beamt_web_in','system_model.aircraft_trim.cruise_1.cruise_1.mass_model.wing_beam_tweb')
caddee_csdl_model.connect('system_model.aircraft_trim.cruise_1.cruise_1.wing_eb_beam_model.wing_beamt_cap_in','system_model.aircraft_trim.cruise_1.cruise_1.mass_model.wing_beam_tcap')



caddee_csdl_model.add_constraint('system_model.aircraft_trim.cruise_1.cruise_1.wing_eb_beam_model.Aframe.new_stress',upper=276E6/1.5,scaler=1E-8)
caddee_csdl_model.add_constraint('system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual', equals=0.)
caddee_csdl_model.add_objective('system_model.aircraft_trim.cruise_1.cruise_1.total_constant_mass_properties.total_mass', scaler=1e-3)

# caddee_csdl_model.add_objective('system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual')

# create and run simulator
sim = Simulator(caddee_csdl_model, analytics=True)
sim.run()
print(sim['system_model.aircraft_trim.cruise_1.cruise_1.wing_eb_beam_model.Aframe.vm_stress'])
print(sim['system_model.aircraft_trim.cruise_1.cruise_1.wing_eb_beam_model.Aframe.wing_beam_forces'])



# sim.compute_total_derivatives()
# sim.check_totals()


prob = CSDLProblem(problem_name='lpc', simulator=sim)
optimizer = SLSQP(prob, maxiter=1000, ftol=1E-5)
optimizer.solve()
optimizer.print_results()

print("I'm done.")
print('trim: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual'])
print('forces: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_forces'])
print('moments:', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_moments'])
print('pitch: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.cruise_1_ac_states_operation.cruise_1_pitch_angle'])
print('rpm: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.rpm'])
print('htail act: ', sim['system_parameterization.ffd_set.rotational_section_properties_model.h_tail_act'])
print('total mass: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.total_constant_mass_properties.total_mass'])
print('wing mass: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.wing_eb_beam_model.Aframe.MassProp.struct_mass']) # wing mass
print('stress: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.wing_eb_beam_model.Aframe.new_stress'])
print('wing beam forces: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.wing_eb_beam_force_mapping.wing_beam_forces'])