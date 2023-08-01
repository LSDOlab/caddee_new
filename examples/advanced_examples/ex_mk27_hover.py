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
wing_primitive_names = list(spatial_rep.get_primitives(search_names=['HorizontalWing']).keys())
MidWing = LiftingSurface(name='MidWing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names) 

wing_primitive_names = list(spatial_rep.get_primitives(search_names=['UpperWing']).keys())
TopWing = LiftingSurface(name='TopWing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)

wing_primitive_names = list(spatial_rep.get_primitives(search_names=['LowerWing']).keys())
BotWing = LiftingSurface(name='BotWing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)

wing_primitive_names = list(spatial_rep.get_primitives(search_names=['TopFrame']).keys())
TopFrame = LiftingSurface(name='TopFrame', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)

wing_primitive_names = list(spatial_rep.get_primitives(search_names=['LowFrame']).keys())
BotFrame = LiftingSurface(name='BotFrame', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)

wing_primitive_names = list(spatial_rep.get_primitives(search_names=['VerticalStabilizer']).keys())
VertStab = LiftingSurface(name='VertStab', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)

# wing plots - check progress
#MidWing.plot()
#TopWing.plot()
#BotWing.plot()
#TopFrame.plot()
#BotFrame.plot()
#VertStab.plot()


# Rotor: pusher - mid, upper, lower propellors (these are mirrored over mid plane for OEI need to remodel VSP to isolate 1 engine?)
pp_disk_prim_names = list(spatial_rep.get_primitives(search_names=['MidProps']).keys())
ppm = cd.Rotor(name='ppm_disk', spatial_representation=spatial_rep, primitive_names=pp_disk_prim_names)

pp_disk_prim_names = list(spatial_rep.get_primitives(search_names=['UpperProps']).keys())
ppu = cd.Rotor(name='ppu_disk', spatial_representation=spatial_rep, primitive_names=pp_disk_prim_names)

pp_disk_prim_names = list(spatial_rep.get_primitives(search_names=['LowerProps']).keys())
ppl = cd.Rotor(name='ppl_disk', spatial_representation=spatial_rep, primitive_names=pp_disk_prim_names)

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
sys_rep.add_component(ppm)
sys_rep.add_component(ppl)
sys_rep.add_component(ppu)



do_plots=False

# region pusher prop (pp) meshes

# disk: middle 
#left
y11 = ppm_left.project(np.array([2.5,-.93,0]), direction=np.array([-1., 0., 0.]), plot=False)
y12 = ppm_left.project(np.array([2.5,-2.57,0]), direction=np.array([-1., 0., 0.]), plot=False)
y21 = ppm_left.project(np.array([2.5,-1.75,0.82]), direction=np.array([-1., 0., 0.]), plot=False)
y22 = ppm_left.project(np.array([2.5,-1.75,-0.82]), direction=np.array([-1., 0., 0.]), plot=False)
ppm_left_plane_y = am.subtract(y11, y12)
ppm_left_plane_x = am.subtract(y21, y22)
ppm_left_origin = ppm.project(np.array([2.5,-1.75,0]), direction=np.array([-1., 0., 0.]))
sys_rep.add_output(f"{ppm_left.parameters['name']}_in_plane_1", ppm_left_plane_y)
sys_rep.add_output(f"{ppm_left.parameters['name']}_in_plane_2", ppm_left_plane_x)
sys_rep.add_output(f"{ppm_left.parameters['name']}_origin", ppm_left_origin)

#right
y11 = ppm_right.project(np.array([2.5,2.57,0]), direction=np.array([-1., 0., 0.]), plot=False)
y21 = ppm_right.project(np.array([2.5,1.75,0.82]), direction=np.array([-1., 0., 0.]), plot=False)
y22 = ppm_right.project(np.array([2.5,1.75,-0.82]), direction=np.array([-1., 0., 0.]), plot=False)
ppm_right_plane_y = am.subtract(y11, y12)
ppm_right_plane_x = am.subtract(y21, y22)
ppm_right_origin = ppm.project(np.array([2.5,1.75,0]), direction=np.array([-1., 0., 0.]))
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
ppu_left_origin = ppm.project(np.array([3.5,-1.15,1.6]), direction=np.array([-.683,0.259,0.683]))
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
ppu_right_origin = ppm.project(np.array([3.5,1.15,1.6]), direction=np.array([-.683,-0.259,0.683]))
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
sys_rep.add_output(f"{ppm.parameters['name']}_in_plane_1", ppl_left_plane_y)
sys_rep.add_output(f"{ppm.parameters['name']}_in_plane_2", ppl_left_plane_x)
sys_rep.add_output(f"{ppm.parameters['name']}_origin", ppl_left_origin)

#right
y11 = ppl_right.project(np.array([1.66,0.775,-1.03]), direction=np.array([-0.75,-0.5,0.433]), plot=False)
y12 = ppl_right.project(np.array([0.84,0.775,-2.45]), direction=np.array([-0.75,-0.5,0.433]), plot=False)
y21 = ppl_right.project(np.array([0.895,0.065,-1.535]), direction=np.array([-0.75,-0.5,0.433]), plot=False)
y22 = ppl_right.project(np.array([1.605,1.485,-1.945]), direction=np.array([-0.75,-0.5,0.433]), plot=False)
ppl_right_plane_y = am.subtract(y11, y12)
ppl_right_plane_x = am.subtract(y21, y22)
ppl_right_origin = ppl_right.project(np.array([1.25,-0.775,-1.74]), direction=np.array([-0.75,-0.5,0.433]))
sys_rep.add_output(f"{ppl_right.parameters['name']}_in_plane_1", pplr_plane_y)
sys_rep.add_output(f"{ppl_right.parameters['name']}_in_plane_2", pplr_plane_x)
sys_rep.add_output(f"{ppl_right.parameters['name']}_origin", pplr_origin)

# endregion

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