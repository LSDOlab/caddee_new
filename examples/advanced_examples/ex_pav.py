# region Imports
import caddee.api as cd
import m3l
from python_csdl_backend import Simulator
from modopt.scipy_library import SLSQP
from modopt.csdl_library import CSDLProblem

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

debug_geom_flag = False
visualize_flag = False


def trim_at_cruise():
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
    if debug_geom_flag:
        wing.plot()
    sys_rep.add_component(wing)

    # Horizontal tail
    tail_primitive_names = list(spatial_rep.get_primitives(search_names=['Stabilizer']).keys())
    htail = cd.LiftingSurface(name='HTail', spatial_representation=spatial_rep, primitive_names=tail_primitive_names)
    if debug_geom_flag:
        htail.plot()
    sys_rep.add_component(htail)

    # Canard
    canard_primitive_names = list(spatial_rep.get_primitives(search_names=['FrontSuport']).keys())
    canard = cd.LiftingSurface(name='Canard', spatial_representation=spatial_rep,
                               primitive_names=canard_primitive_names)
    if debug_geom_flag:
        canard.plot()
    sys_rep.add_component(canard)
    # endregion

    # region Rotors
    # Pusher prop
    pp_disk_prim_names = list(spatial_rep.get_primitives(search_names=['PropPusher']).keys())
    pp_disk = cd.Rotor(name='pp_disk', spatial_representation=spatial_rep, primitive_names=pp_disk_prim_names)
    if debug_geom_flag:
        pp_disk.plot()
    sys_rep.add_component(pp_disk)
    # endregion

    # endregion

    # region Actuations
    # Tail FFD
    htail_geometry_primitives = htail.get_geometry_primitives()
    htail_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(
        htail_geometry_primitives,
        num_control_points=(11, 2, 2), order=(4, 2, 2),
        xyz_to_uvw_indices=(1, 0, 2)
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
        ffd_blocks={htail_ffd_block.name: htail_ffd_block}
    )
    sys_param.add_geometry_parameterization(ffd_set)
    sys_param.setup()
    # endregion

    # region Meshes

    # region Wing
    num_wing_vlm = 21
    num_chordwise_vlm = 5
    point00 = np.array([8.796, 14.000, 1.989])  # * ft2m # Right tip leading edge
    point01 = np.array([11.300, 14.000, 1.989])  # * ft2m # Right tip trailing edge
    point10 = np.array([8.800, 0.000, 1.989])  # * ft2m # Center Leading Edge
    point11 = np.array([15.170, 0.000, 1.989])  # * ft2m # Center Trailing edge
    point20 = np.array([8.796, -14.000, 1.989])  # * ft2m # Left tip leading edge
    point21 = np.array([11.300, -14.000, 1.989])  # * ft2m # Left tip

    leading_edge_points = np.concatenate(
        (np.linspace(point00, point10, int(num_wing_vlm / 2 + 1))[0:-1, :],
         np.linspace(point10, point20, int(num_wing_vlm / 2 + 1))),
        axis=0)
    trailing_edge_points = np.concatenate(
        (np.linspace(point01, point11, int(num_wing_vlm / 2 + 1))[0:-1, :],
         np.linspace(point11, point21, int(num_wing_vlm / 2 + 1))),
        axis=0)

    leading_edge = wing.project(leading_edge_points, direction=np.array([-1., 0., 0.]), plot=debug_geom_flag)
    trailing_edge = wing.project(trailing_edge_points, direction=np.array([1., 0., 0.]), plot=debug_geom_flag)

    # Chord Surface
    wing_chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
    if debug_geom_flag:
        spatial_rep.plot_meshes([wing_chord_surface])

    # Upper and lower surface
    wing_upper_surface_wireframe = wing.project(wing_chord_surface.value + np.array([0., 0., 0.5]),
                                                direction=np.array([0., 0., -1.]), grid_search_n=25,
                                                plot=debug_geom_flag, max_iterations=200)
    wing_lower_surface_wireframe = wing.project(wing_chord_surface.value - np.array([0., 0., 0.5]),
                                                direction=np.array([0., 0., 1.]), grid_search_n=25,
                                                plot=debug_geom_flag, max_iterations=200)

    # Chamber surface
    wing_camber_surface = am.linspace(wing_upper_surface_wireframe, wing_lower_surface_wireframe, 1)
    wing_vlm_mesh_name = 'wing_vlm_mesh'
    sys_rep.add_output(wing_vlm_mesh_name, wing_camber_surface)
    if debug_geom_flag:
        spatial_rep.plot_meshes([wing_camber_surface])

    # OML mesh
    wing_oml_mesh = am.vstack((wing_upper_surface_wireframe, wing_lower_surface_wireframe))
    wing_oml_mesh_name = 'wing_oml_mesh'
    sys_rep.add_output(wing_oml_mesh_name, wing_oml_mesh)
    if debug_geom_flag:
        spatial_rep.plot_meshes([wing_oml_mesh])
    # endregion

    # region Tail
    num_htail_vlm = 13
    num_chordwise_vlm = 5
    point00 = np.array([20.713 - 4., 8.474 + 1.5, 0.825 + 1.5])  # * ft2m # Right tip leading edge
    point01 = np.array([22.916, 8.474, 0.825])  # * ft2m # Right tip trailing edge
    point10 = np.array([18.085, 0.000, 0.825])  # * ft2m # Center Leading Edge
    point11 = np.array([23.232, 0.000, 0.825])  # * ft2m # Center Trailing edge
    point20 = np.array([20.713 - 4., -8.474 - 1.5, 0.825 + 1.5])  # * ft2m # Left tip leading edge
    point21 = np.array([22.916, -8.474, 0.825])  # * ft2m # Left tip trailing edge

    leading_edge_points = np.linspace(point00, point20, num_htail_vlm)
    trailing_edge_points = np.linspace(point01, point21, num_htail_vlm)

    leading_edge = htail.project(leading_edge_points, direction=np.array([0., 0., -1.]), plot=debug_geom_flag)
    trailing_edge = htail.project(trailing_edge_points, direction=np.array([1., 0., 0.]), plot=debug_geom_flag)

    # Chord Surface
    htail_chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
    if debug_geom_flag:
        spatial_rep.plot_meshes([htail_chord_surface])

    # Upper and lower surface
    htail_upper_surface_wireframe = htail.project(htail_chord_surface.value + np.array([0., 0., 0.5]),
                                                  direction=np.array([0., 0., -1.]), grid_search_n=25,
                                                  plot=debug_geom_flag, max_iterations=200)
    htail_lower_surface_wireframe = htail.project(htail_chord_surface.value - np.array([0., 0., 0.5]),
                                                  direction=np.array([0., 0., 1.]), grid_search_n=25,
                                                  plot=debug_geom_flag, max_iterations=200)

    # Chamber surface
    htail_camber_surface = am.linspace(htail_upper_surface_wireframe, htail_lower_surface_wireframe, 1)
    htail_vlm_mesh_name = 'htail_vlm_mesh'
    sys_rep.add_output(htail_vlm_mesh_name, htail_camber_surface)
    if debug_geom_flag:
        spatial_rep.plot_meshes([htail_camber_surface])

    # OML mesh
    htail_oml_mesh = am.vstack((htail_upper_surface_wireframe, htail_lower_surface_wireframe))
    htail_oml_mesh_name = 'htail_oml_mesh'
    sys_rep.add_output(htail_oml_mesh_name, htail_oml_mesh)
    if debug_geom_flag:
        spatial_rep.plot_meshes([htail_oml_mesh])
    # endregion

    # region Canard

    # endregion

    if visualize_flag:
        spatial_rep.plot_meshes([wing_camber_surface, htail_camber_surface])

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

    # region Mission

    design_scenario = cd.DesignScenario(name='aircraft_trim')

    # region Cruise condition
    cruise_model = m3l.Model()
    cruise_condition = cd.CruiseCondition(name="cruise_1")
    cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()
    cruise_condition.set_module_input(name='altitude', val=600 * ft2m)
    cruise_condition.set_module_input(name='mach_number', val=0.145972)  # 112 mph = 0.145972 Mach
    cruise_condition.set_module_input(name='range', val=80467.2)  # 50 miles = 80467.2 m
    cruise_condition.set_module_input(name='pitch_angle', val=np.deg2rad(0), dv_flag=True, lower=np.deg2rad(-10),
                                      upper=np.deg2rad(10))
    cruise_condition.set_module_input(name='flight_path_angle', val=0)
    cruise_condition.set_module_input(name='roll_angle', val=0)
    cruise_condition.set_module_input(name='yaw_angle', val=0)
    cruise_condition.set_module_input(name='wind_angle', val=0)
    cruise_condition.set_module_input(name='observer_location', val=np.array([0, 0, 600 * ft2m]))

    cruise_ac_states = cruise_condition.evaluate_ac_states()
    cruise_model.register_output(cruise_ac_states)

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
    bem_model.set_module_input('rpm', val=4000)
    bem_model.set_module_input('propeller_radius', val=3.97727 / 2 * ft2m)
    bem_model.set_module_input('thrust_vector', val=np.array([1., 0., 0.]))
    bem_model.set_module_input('thrust_origin', val=np.array([19.700, 0., 2.625]))
    bem_model.set_module_input('chord_cp', val=np.linspace(0.2, 0.05, 4),
                               dv_flag=True,
                               upper=np.array([0.25, 0.25, 0.25, 0.25]), lower=np.array([0.05, 0.05, 0.05, 0.05]),
                               scaler=1
                               )
    bem_model.set_module_input('twist_cp', val=np.deg2rad(np.linspace(65, 15, 4)),
                               dv_flag=True,
                               lower=np.deg2rad(5), upper=np.deg2rad(85), scaler=1
                               )
    bem_forces, bem_moments, _, _, _ = bem_model.evaluate(ac_states=cruise_ac_states)
    cruise_model.register_output(bem_forces)
    cruise_model.register_output(bem_moments)
    # endregion

    # region Inertial loads
    inertial_loads_model = cd.InertialLoadsM3L(load_factor=1.)
    inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg, totoal_mass=total_mass,
                                                                      ac_states=cruise_ac_states)
    cruise_model.register_output(inertial_forces)
    cruise_model.register_output(inertial_moments)
    # endregion

    # region Aerodynamics
    vlm_model = VASTFluidSover(
        surface_names=[
            wing_vlm_mesh_name,
            htail_vlm_mesh_name,
        ],
        surface_shapes=[
            (1,) + wing_camber_surface.evaluate().shape[1:],
            (1,) + htail_camber_surface.evaluate().shape[1:],
        ],
        fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
        mesh_unit='ft',
        cl0=[0.55, 0.0]
    )
    vlm_panel_forces, vlm_forces, vlm_moments = vlm_model.evaluate(ac_states=cruise_ac_states)
    cruise_model.register_output(vlm_forces)
    cruise_model.register_output(vlm_moments)
    # endregion

    # Total loads
    total_forces_moments_model = cd.TotalForcesMomentsM3L()
    total_forces, total_moments = total_forces_moments_model.evaluate(
        inertial_forces, inertial_moments,
        vlm_forces, vlm_moments,
        bem_forces, bem_moments
    )
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

    caddee_csdl_model.create_input(name='h_tail_act', val=np.deg2rad(0.))
    caddee_csdl_model.add_design_variable(dv_name='h_tail_act', lower=np.deg2rad(-10), upper=np.deg2rad(10), scaler=1.)

    # region Optimization Setup
    caddee_csdl_model.add_objective('system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual')
    caddee_csdl_model.add_constraint(
        name='system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.induced_velocity_model.eta',
        equals=0.8)
    caddee_csdl_model.add_constraint(
        name='system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_mesh_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.L_over_D',
        equals=8.,
        scaler=1e-1
    )
    # endregion

    # Create and run simulator
    sim = Simulator(caddee_csdl_model, analytics=True)
    sim.run()
    # sim.compute_total_derivatives()
    # sim.check_totals()

    print('Total forces: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_forces'])
    print('Total moments:', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_moments'])

    # system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.induced_velocity_model.FOM

    prob = CSDLProblem(problem_name='lpc', simulator=sim)
    optimizer = SLSQP(prob, maxiter=1000, ftol=1E-10)
    optimizer.solve()
    optimizer.print_results()

    print('Trim residual: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual'])
    print('Trim forces: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_forces'])
    print('Trim moments:', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_moments'])
    print('Pitch: ', np.rad2deg(
        sim['system_model.aircraft_trim.cruise_1.cruise_1.cruise_1_ac_states_operation.cruise_1_pitch_angle']))
    print('RPM: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.rpm'])
    print('Horizontal tail actuation: ',
          np.rad2deg(sim['system_parameterization.ffd_set.rotational_section_properties_model.h_tail_act']))

    print(sim['system_model.aircraft_trim.cruise_1.cruise_1.pp_disk_bem_model.induced_velocity_model.eta'])
    print(sim[
              'system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_mesh_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.L_over_D'])


    return


def trim_at_hover():
    caddee = cd.CADDEE()
    caddee.system_model = system_model = cd.SystemModel()
    caddee.system_representation = sys_rep = cd.SystemRepresentation()
    caddee.system_parameterization = sys_param = cd.SystemParameterization(system_representation=sys_rep)

    # region Geometry
    file_name = 'pav.stp'

    spatial_rep = sys_rep.spatial_representation
    spatial_rep.import_file(file_name=GEOMETRY_FILES_FOLDER / file_name)
    spatial_rep.refit_geometry(file_name=GEOMETRY_FILES_FOLDER / file_name)

    # region Rotors
    # region Lift rotor: Right 1
    lr_r1_disk_prim_names = list(spatial_rep.get_primitives(search_names=['PropRight1']).keys())
    lr_r1_disk = cd.Rotor(name='lr_r1_disk', 
                          spatial_representation=spatial_rep, 
                          primitive_names=lr_r1_disk_prim_names)
    if debug_geom_flag:
        lr_r1_disk.plot()
    sys_rep.add_component(lr_r1_disk)
    # endregion

    # region Lift rotor: Right 2
    lr_r2_disk_prim_names = list(spatial_rep.get_primitives(search_names=['PropRight2']).keys())
    lr_r2_disk = cd.Rotor(name='lr_r2_disk', 
                          spatial_representation=spatial_rep, 
                          primitive_names=lr_r2_disk_prim_names)
    if debug_geom_flag:
        lr_r2_disk.plot()
    sys_rep.add_component(lr_r2_disk)
    # endregion

    # region Lift rotor: Right 3
    lr_r3_disk_prim_names = list(spatial_rep.get_primitives(search_names=['PropRight3']).keys())
    lr_r3_disk = cd.Rotor(name='lr_r3_disk', 
                          spatial_representation=spatial_rep, 
                          primitive_names=lr_r3_disk_prim_names)
    if debug_geom_flag:
        lr_r3_disk.plot()
    sys_rep.add_component(lr_r3_disk)
    # endregion

    # region Lift rotor: Right 4
    lr_r4_disk_prim_names = list(spatial_rep.get_primitives(search_names=['PropRight4']).keys())
    lr_r4_disk = cd.Rotor(name='lr_r4_disk', 
                          spatial_representation=spatial_rep, 
                          primitive_names=lr_r4_disk_prim_names)
    if debug_geom_flag:
        lr_r4_disk.plot()
    sys_rep.add_component(lr_r4_disk)
    # endregion

    # region Lift rotor: Left 1
    lr_l1_disk_prim_names = list(spatial_rep.get_primitives(search_names=['PropLeft1']).keys())
    lr_l1_disk = cd.Rotor(name='lr_l1_disk', 
                          spatial_representation=spatial_rep, 
                          primitive_names=lr_l1_disk_prim_names)
    if debug_geom_flag:
        lr_l1_disk.plot()
    sys_rep.add_component(lr_l1_disk)
    # endregion
    
    # region Lift rotor: Left 2
    lr_l2_disk_prim_names = list(spatial_rep.get_primitives(search_names=['PropLeft2']).keys())
    lr_l2_disk = cd.Rotor(name='lr_l2_disk', 
                          spatial_representation=spatial_rep, 
                          primitive_names=lr_l2_disk_prim_names)
    if debug_geom_flag:
        lr_l2_disk.plot()
    sys_rep.add_component(lr_l2_disk)
    # endregion

    # region Lift rotor: Left 3
    lr_l3_disk_prim_names = list(spatial_rep.get_primitives(search_names=['PropLeft3']).keys())
    lr_l3_disk = cd.Rotor(name='lr_l3_disk', 
                          spatial_representation=spatial_rep, 
                          primitive_names=lr_l3_disk_prim_names)
    if debug_geom_flag:
        lr_l3_disk.plot()
    sys_rep.add_component(lr_l3_disk)
    # endregion

    # region Lift rotor: Left 4
    lr_l4_disk_prim_names = list(spatial_rep.get_primitives(search_names=['PropLeft4']).keys())
    lr_l4_disk = cd.Rotor(name='lr_l4_disk', 
                          spatial_representation=spatial_rep, 
                          primitive_names=lr_l4_disk_prim_names)
    if debug_geom_flag:
        lr_l4_disk.plot()
    sys_rep.add_component(lr_l4_disk)
    # endregion

    # endregion
    
    # endregion

    # region Sizing
    pav_wt = PavMassProperties()
    mass, cg, I = pav_wt.evaluate()

    total_mass_properties = cd.TotalMassPropertiesM3L()
    total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass, cg, I)
    # endregion

    # region Mission

    design_scenario = cd.DesignScenario(name='aircraft_trim')

    # region Hover condition
    hover_model = m3l.Model()
    hover_condition = cd.HoverCondition(name="hover")
    hover_condition.atmosphere_model = cd.SimpleAtmosphereModel()
    hover_condition.set_module_input(name='altitude', val=50 * ft2m)
    hover_condition.set_module_input(name='hover_time', val=30)  # 30 seconds
    hover_condition.set_module_input(name='observer_location', val=np.array([0, 0, 50 * ft2m]))

    hover_ac_states = hover_condition.evaluate_ac_states()
    hover_model.register_output(hover_ac_states)

    # region Propulsion Loads: Lift Rotor: Right 1
    lr_r1_bem_mesh = BEMMesh(
        airfoil='NACA_4412',
        num_blades=2,
        num_radial=25,
        use_airfoil_ml=False,
        use_rotor_geometry=False,
        mesh_units='ft',
        chord_b_spline_rep=True,
        twist_b_spline_rep=True
    )
    lr_r1_bem_model = BEM(disk_prefix='lr_r1_disk', blade_prefix='lr_r1', 
                    component=lr_r1_disk, 
                    mesh=lr_r1_bem_mesh)
    lr_r1_bem_model.set_module_input('rpm', val=2000)
    lr_r1_bem_model.set_module_input('propeller_radius', val=5.17045 / 2 * ft2m)
    lr_r1_bem_model.set_module_input('thrust_vector', val=np.array([0., 0., -1.]))
    lr_r1_bem_model.set_module_input('thrust_origin', val=np.array([-3.000, 5.313, -0.530]))
    lr_r1_bem_model.set_module_input('chord_cp', val=np.linspace(0.2, 0.05, 4),
                               dv_flag=True,
                               upper=np.array([0.25, 0.25, 0.25, 0.25]), lower=np.array([0.05, 0.05, 0.05, 0.05]),
                               scaler=1
                               )
    lr_r1_bem_model.set_module_input('twist_cp', val=np.deg2rad(np.linspace(65, 15, 4)),
                               dv_flag=True,
                               lower=np.deg2rad(5), upper=np.deg2rad(85), scaler=1
                               )
    lr_r1_bem_forces, lr_r1_bem_moments, _, _, _ = lr_r1_bem_model.evaluate(ac_states=hover_ac_states)
    hover_model.register_output(lr_r1_bem_forces)
    hover_model.register_output(lr_r1_bem_moments)
    # endregion

    # Total loads
    total_forces_moments_model = cd.TotalForcesMomentsM3L()
    total_forces, total_moments = total_forces_moments_model.evaluate(
        lr_r1_bem_forces, lr_r1_bem_moments
    )
    hover_model.register_output(total_forces)
    hover_model.register_output(total_moments)

    # Equations of motions
    eom_m3l_model = cd.EoMM3LEuler6DOF()
    trim_residual = eom_m3l_model.evaluate(
        total_mass=total_mass,
        total_cg_vector=total_cg,
        total_inertia_tensor=total_inertia,
        total_forces=total_forces,
        total_moments=total_moments,
        ac_states=hover_ac_states
    )

    hover_model.register_output(trim_residual)

    # Add hover m3l model to hover condition
    hover_condition.add_m3l_model('hover_model', hover_model)

    # Add design condition to design scenario
    design_scenario.add_design_condition(hover_condition)
    # endregion

    system_model.add_design_scenario(design_scenario=design_scenario)
    # endregion

    caddee_csdl_model = caddee.assemble_csdl()

    # Create and run simulator
    sim = Simulator(caddee_csdl_model, analytics=True)
    sim.run()

    print('Total forces: ', sim['system_model.aircraft_trim.hover.hover.euler_eom_gen_ref_pt.total_forces'])
    print('Total moments:', sim['system_model.aircraft_trim.hover.hover.euler_eom_gen_ref_pt.total_moments'])

    print('Lift rotor right 1 RPM: ', sim['system_model.aircraft_trim.hover.hover.lr_r1_disk_bem_model.rpm'])
    print('Lift rotor right 1 FoM: ', sim['system_model.aircraft_trim.hover.hover.lr_r1_disk_bem_model.induced_velocity_model.FOM'])

    return



if __name__ == '__main__':
    # trim_at_cruise()
    trim_at_hover()
