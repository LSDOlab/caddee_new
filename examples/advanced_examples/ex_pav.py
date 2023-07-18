# region Imports
import caddee.api as cd
import m3l
from python_csdl_backend import Simulator

# Geometry
from caddee.core.caddee_core.system_representation.component.component import LiftingSurface, Component
import array_mapper as am

# Solvers
from lsdo_rotor.core.BEM_caddee.BEM_caddee import BEM, BEMMesh
from VAST.core.vast_solver import VASTFluidSover
from VAST.core.fluid_problem import FluidProblem
from caddee.utils.aircraft_models.pav.pav_weight import PavMassProperties

from caddee import GEOMETRY_FILES_FOLDER

import numpy as np
# endregion


ft2m = 0.3048

plots_flag = False

caddee = cd.CADDEE()
caddee.system_model = system_model = cd.SystemModel()
caddee.system_representation = sys_rep = cd.SystemRepresentation()
caddee.system_parameterization = sys_param = cd.SystemParameterization(system_representation=sys_rep)

# region Geometry
file_name = 'pav.stp'

spatial_rep = sys_rep.spatial_representation
spatial_rep.import_file(file_name=GEOMETRY_FILES_FOLDER / file_name)
spatial_rep.refit_geometry(file_name=GEOMETRY_FILES_FOLDER / file_name)

# region Lifting surfaces
# Wing
wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Wing']).keys())
wing = LiftingSurface(name='Wing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)
if plots_flag:
    wing.plot()
sys_rep.add_component(wing)

# Horizontal tail
tail_primitive_names = list(spatial_rep.get_primitives(search_names=['Stabilizer']).keys())
htail = cd.LiftingSurface(name='HTail', spatial_representation=spatial_rep, primitive_names=tail_primitive_names)
if plots_flag:
    htail.plot()
sys_rep.add_component(htail)
# endregion

# region Rotors
# Pusher prop
pp_disk_prim_names = list(spatial_rep.get_primitives(search_names=['PropPusher']).keys())
pp_disk = cd.Rotor(name='pp_disk', spatial_representation=spatial_rep, primitive_names=pp_disk_prim_names)
if plots_flag:
    pp_disk.plot()
sys_rep.add_component(pp_disk)
# endregion

# endregion

# region Actuations
# Tail FFD
htail_geometry_primitives = htail.get_geometry_primitives()
htail_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(
    htail_geometry_primitives,
    num_control_points=(11, 2, 2), order=(4,2,2),
    xyz_to_uvw_indices=(1,0,2)
)
htail_ffd_block = cd.SRBGFFDBlock(name='htail_ffd_block',
                                  primitive=htail_ffd_bspline_volume,
                                  embedded_entities=htail_geometry_primitives)
htail_ffd_block.add_scale_v(name='htail_linear_taper',
                            order=2, num_dof=3, value=np.array([0., 0., 0.]),
                            cost_factor=1.)
htail_ffd_block.add_rotation_u(name='htail_twist_distribution',
                               connection_name='h_tail_act', order=1,
                               num_dof=1, value=np.array([np.deg2rad(1.75)]))
ffd_set = cd.SRBGFFDSet(
    name='ffd_set',
    ffd_blocks={htail_ffd_block.name : htail_ffd_block}
)
sys_param.add_geometry_parameterization(ffd_set)
sys_param.setup()
# endregion

# region Meshes

# region Wing
num_wing_vlm = 21
num_chordwise_vlm = 5
point00 = np.array([8.167, 13.997,  1.989 + 0.1]) # * ft2m # Right tip leading edge
point01 = np.array([10.565, 13.997,  1.989]) # * ft2m # Right tip trailing edge
point10 = np.array([8.171, 0.0000,  1.989 + 0.1]) # * ft2m # Center Leading Edge
point11 = np.array([13.549, 0.0000,  1.989]) # * ft2m # Center Trailing edge
point20 = np.array([8.167, -13.997, 1.989 + 0.1]) # * ft2m # Left tip leading edge
point21 = np.array([10.565, -13.997, 1.989]) # * ft2m # Left tip trailing edge

leading_edge_points = np.concatenate(
    (np.linspace(point00, point10, int(num_wing_vlm/2+1))[0:-1,:],
     np.linspace(point10, point20, int(num_wing_vlm/2+1))),
    axis=0)
trailing_edge_points = np.concatenate(
    (np.linspace(point01, point11, int(num_wing_vlm/2+1))[0:-1,:],
     np.linspace(point11, point21, int(num_wing_vlm/2+1))),
    axis=0)

leading_edge = wing.project(leading_edge_points, direction=np.array([-1., 0., 0.]), plot=plots_flag)
trailing_edge = wing.project(trailing_edge_points, direction=np.array([1., 0., 0.]), plot=plots_flag)

# Chord Surface
wing_chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
if plots_flag:
    spatial_rep.plot_meshes([wing_chord_surface])

# Upper and lower surface
wing_upper_surface_wireframe = wing.project(wing_chord_surface.value + np.array([0., 0., 0.5]),
                                            direction=np.array([0., 0., -1.]), grid_search_n=25,
                                            plot=plots_flag, max_iterations=200)
wing_lower_surface_wireframe = wing.project(wing_chord_surface.value - np.array([0., 0., 0.5]),
                                            direction=np.array([0., 0., 1.]), grid_search_n=25,
                                            plot=plots_flag, max_iterations=200)

# Chamber surface
wing_camber_surface = am.linspace(wing_upper_surface_wireframe, wing_lower_surface_wireframe, 1)
wing_vlm_mesh_name = 'wing_vlm_mesh'
sys_rep.add_output(wing_vlm_mesh_name, wing_camber_surface)
if plots_flag:
    spatial_rep.plot_meshes([wing_camber_surface])

# OML mesh
wing_oml_mesh = am.vstack((wing_upper_surface_wireframe, wing_lower_surface_wireframe))
wing_oml_mesh_name = 'wing_oml_mesh'
sys_rep.add_output(wing_oml_mesh_name, wing_oml_mesh)
if plots_flag:
    spatial_rep.plot_meshes([wing_oml_mesh])
# endregion

# region Tail
# endregion

# region Canard
# endregion

# region Pusher prop
# y11 = pp_disk.project(np.array([23.500 + 0.1, 0.00, 0.800]), direction=np.array([-1., 0., 0.]), plot=False)
# y12 = pp_disk.project(np.array([23.500 + 0.1, 0.00, 5.800]), direction=np.array([-1., 0., 0.]), plot=False)
# y21 = pp_disk.project(np.array([23.500 + 0.1, -2.500, 3.300]), direction=np.array([-1., 0., 0.]), plot=False)
# y22 = pp_disk.project(np.array([23.500 + 0.1, 2.500, 3.300]), direction=np.array([-1., 0., 0.]), plot=False)
# pp_disk_in_plane_y = am.subtract(y11, y12)
# pp_disk_in_plane_x = am.subtract(y21, y22)
# pp_disk_origin = pp_disk.project(np.array([32.625, 0., 7.79]), direction=np.array([-1., 0., 0.]))
# sys_rep.add_output(f"{pp_disk.parameters['name']}_in_plane_1", pp_disk_in_plane_y)
# sys_rep.add_output(f"{pp_disk.parameters['name']}_in_plane_2", pp_disk_in_plane_x)
# sys_rep.add_output(f"{pp_disk.parameters['name']}_origin", pp_disk_origin)
# endregion




# endregion

# region Sizing
pav_wt = PavMassProperties()
mass, cg, I = pav_wt.evaluate()

total_mass_properties = cd.TotalMassPropertiesM3L()
total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass, cg, I)


# endregion

# region Physics models

# region Propulsion
pusher_bem_mesh = BEMMesh(
    airfoil='NACA_4412',
    num_blades=5,
    num_radial=25,
    use_airfoil_ml=False,
    use_rotor_geometry=False,
    mesh_units='ft',
    chord_b_spline_rep=True,
    twist_b_spline_rep=True
)

bem_model = BEM(disk_prefix='pp_disk', blade_prefix='pp', component=pp_disk, mesh=pusher_bem_mesh)
bem_model.set_module_input('rpm', val=4000, dv_flag=True, lower=3500, upper=4500, scaler=1e-3)
bem_model.set_module_input('propeller_radius', val=2*ft2m)  # 2 ft radius propeller
bem_model.set_module_input('thrust_vector', val=np.array([1., 0., 0.]))
bem_model.set_module_input('thrust_origin', val=np.array([18.693, 0., 2.625]))
bem_model.set_module_input('chord_cp', val=np.linspace(0.2, 0.05, 4),
                           dv_flag=True,
                           upper=np.array([0.25, 0.25, 0.25, 0.25]), lower=np.array([0.05, 0.05, 0.05, 0.05]), scaler=1
                           )
bem_model.set_module_input('twist_cp', val=np.deg2rad(np.linspace(65, 15, 4)),
                           dv_flag=True,
                           lower=np.deg2rad(5), upper=np.deg2rad(85), scaler=1
                           )

# endregion

# region Aerodynamics
vlm_model = VASTFluidSover(
    surface_names=[
        wing_vlm_mesh_name,
    ],
    surface_shapes=[
        (1, ) + wing_camber_surface.evaluate().shape[1:],
        ],
    fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
    mesh_unit='ft',
    cl0=[0.0, ]
)
# endregion

# endregion

# region Mission

design_scenario = cd.DesignScenario(name='aircraft_trim')

# region Cruise condition
cruise_model = m3l.Model()
cruise_condition = cd.CruiseCondition(name="cruise_1")
cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()
cruise_condition.set_module_input(name='altitude', val=600*ft2m)
cruise_condition.set_module_input(name='mach_number', val=0.17)
cruise_condition.set_module_input(name='range', val=40000)
cruise_condition.set_module_input(name='pitch_angle', val=np.deg2rad(0), dv_flag=True, lower=0., upper=np.deg2rad(10))
cruise_condition.set_module_input(name='flight_path_angle', val=0)
cruise_condition.set_module_input(name='roll_angle', val=0)
cruise_condition.set_module_input(name='yaw_angle', val=0)
cruise_condition.set_module_input(name='wind_angle', val=0)
cruise_condition.set_module_input(name='observer_location', val=np.array([0, 0, 600*ft2m]))

cruise_ac_states = cruise_condition.evaluate_ac_states()
cruise_model.register_output(cruise_ac_states)

# Propulsion loads
bem_forces, bem_moments, _, _, _ = bem_model.evaluate(ac_states=cruise_ac_states)
cruise_model.register_output(bem_forces)
cruise_model.register_output(bem_moments)

# Inertial loads
inertial_loads_model = cd.InertialLoadsM3L(load_factor=1.)
inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg, totoal_mass=total_mass, ac_states=cruise_ac_states)
cruise_model.register_output(inertial_forces)
cruise_model.register_output(inertial_moments)

# Aerodynamic loads
vlm_panel_forces, vlm_force, vlm_moment  = vlm_model.evaluate(ac_states=cruise_ac_states)
cruise_model.register_output(vlm_force)
cruise_model.register_output(vlm_moment)

# Total loads
total_forces_moments_model = cd.TotalForcesMomentsM3L()
total_forces, total_moments = total_forces_moments_model.evaluate(vlm_force, vlm_moment)
cruise_model.register_output(total_forces)
cruise_model.register_output(total_moments)

# Equations of motions
eom_m3l_model = cd.EoMM3LEuler6DOF()
trim_residual = eom_m3l_model.evaluate(
    total_mass=total_mass,
    total_cg_vector=total_cg,
    total_inertia_tensor=total_inertia,
    total_forces=total_forces,
    total_moments=total_moments,
    ac_states=cruise_ac_states
)

cruise_model.register_output(trim_residual)

# Add cruise m3l model to cruise condition
cruise_condition.add_m3l_model('cruise_model', cruise_model)

# Add design condition to design scenario
design_scenario.add_design_condition(cruise_condition)
# endregion

system_model.add_design_scenario(design_scenario=design_scenario)
# endregion

caddee_csdl_model = caddee.assemble_csdl()

# region Give a constraint on L/D

# endregion

# create and run simulator
sim = Simulator(caddee_csdl_model, analytics=True)
sim.run()

# print('Pusher prop RPM:', sim['system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.BEM_external_inputs_model.rpm'])
print('Total forces: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_forces'])
print('Total moments:', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_moments'])

# system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.induced_velocity_model.FOM