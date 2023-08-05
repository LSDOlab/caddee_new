# region Imports
import caddee.api as cd
import m3l
from python_csdl_backend import Simulator
# from modopt.snopt_library import SNOPT
from modopt.scipy_library import SLSQP
from modopt.csdl_library import CSDLProblem
import csdl
import lsdo_geo as lg

# Geometry
import array_mapper as am
from caddee.core.caddee_core.system_representation.component.component import LiftingSurface, Component
from caddee.core.primitives.bsplines.bspline_functions import create_bspline_from_corners

# Solvers
import aframe.core.beam_module as ebbeam
from VAST.core.vast_solver import VASTFluidSover
from VAST.core.fluid_problem import FluidProblem
from VAST.core.generate_mappings_m3l import VASTNodalForces
from caddee.utils.aircraft_models.tbw.tbw_weights import TBWMassProperties
from caddee.utils.aircraft_models.tbw.tbw_propulsion import tbwPropulsionModel
from caddee.utils.aircraft_models.tbw.tbw_viscous_drag import TbwViscousDragModel


from caddee import GEOMETRY_FILES_FOLDER

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
# endregion

debug_geom_flag = False

# def trim_at_1g():
#
#     caddee = cd.CADDEE()
#     caddee.system_model = system_model = cd.SystemModel()
#     caddee.system_representation = sys_rep = cd.SystemRepresentation()
#     caddee.system_parameterization = sys_param = cd.SystemParameterization(system_representation=sys_rep)
#
#     # region Geometry
#     file_name = 'tbw.stp'
#
#     spatial_rep = sys_rep.spatial_representation
#     spatial_rep.import_file(file_name= GEOMETRY_FILES_FOLDER/ file_name)
#     spatial_rep.refit_geometry(file_name=GEOMETRY_FILES_FOLDER / file_name)
#
#     # region Lifting Surfaces
#     # wing
#     wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Wing']).keys())
#     wing = LiftingSurface(name='wing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)
#     if debug_geom_flag:
#         wing.plot()
#     sys_rep.add_component(wing)
#
#     # Horizontal tail
#     tail_primitive_names = list(spatial_rep.get_primitives(search_names=['Htail']).keys())
#     htail = cd.LiftingSurface(name='h_tail', spatial_representation=spatial_rep, primitive_names=tail_primitive_names)
#     if debug_geom_flag:
#         htail.plot()
#     sys_rep.add_component(htail)
#
#     #Strut
#     strut_primitive_names = list(spatial_rep.get_primitives(search_names=['Strut']).keys())
#     strut = cd.LiftingSurface(name='strut', spatial_representation=spatial_rep, primitive_names=strut_primitive_names)
#     if debug_geom_flag:
#         strut.plot()
#     sys_rep.add_component(strut)
#
#     #jury
#     jury_primitive_names = list(spatial_rep.get_primitives(search_names=['Jury']).keys())
#     jury = cd.LiftingSurface(name='jury', spatial_representation=spatial_rep, primitive_names=jury_primitive_names)
#     if debug_geom_flag:
#         jury.plot()
#     sys_rep.add_component(jury)
#     # endregion
#     # endregion
#
#     # region Actuations
#     # Tail FFD
#     htail_geometry_primitives = htail.get_geometry_primitives()
#     htail_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(
#         htail_geometry_primitives,
#         num_control_points=(11, 2, 2), order=(4,2,2),
#         xyz_to_uvw_indices=(1,0,2))
#     htail_ffd_block = cd.SRBGFFDBlock(name='htail_ffd_block',
#                                     primitive=htail_ffd_bspline_volume,
#                                     embedded_entities=htail_geometry_primitives)
#     htail_ffd_block.add_scale_v(
#         name='htail_linear_taper',
#         order=2, num_dof=3, value=np.array([0., 0., 0.]),
#         cost_factor=1.)
#     htail_ffd_block.add_rotation_u(name='htail_twist_distribution',
#                                 connection_name='h_tail_act', order=1,
#                                 num_dof=1, value=np.array([np.deg2rad(1.75)]))
#     # # NOTE: line above is performaing actuation- change when actuations are ready
#
#     ffd_set = cd.SRBGFFDSet(
#         name='ffd_set',
#         ffd_blocks={htail_ffd_block.name : htail_ffd_block}
#     )
#     sys_param.add_geometry_parameterization(ffd_set)
#     sys_param.setup()
#
#     # endregion
#
#     # region meshes
#
#     num_spanwise = 21
#     num_spanwise_strut = num_spanwise
#     num_chordwise = 5
#     num_chordwise_strut = num_chordwise
#
#     # region wing mesh
#     mesh_flag_wing = False
#     num_wing_vlm = num_spanwise
#     num_chordwise_vlm = num_chordwise
#     point00 = np.array([68.035, 85.291, 4.704 + 0.1]) # * ft2m # Right tip leading edge
#     point01 = np.array([71.790, 85.291, 4.708 + 0.1]) # * ft2m # Right tip trailing edge
#     point10 = np.array([47.231,    0.000, 6.937 + 0.1]) # * ft2m # Center Leading Edge
#     point11 = np.array([57.953,   0.000, 6.574 + 0.1]) # * ft2m # Center Trailing edge
#     point20 = np.array([68.035, -85.291, 4.704 + 0.1]) # * ft2m # Left tip leading edge
#     point21 = np.array([71.790, -85.291, 4.708 + 0.1]) # * ft2m # Left tip trailing edge
#
#     do_plots = False
#
#     leading_edge_points = np.concatenate((np.linspace(point00, point10, int(num_wing_vlm/2+1))[0:-1,:], np.linspace(point10, point20, int(num_wing_vlm/2+1))), axis=0)
#     trailing_edge_points = np.concatenate((np.linspace(point01, point11, int(num_wing_vlm/2+1))[0:-1,:], np.linspace(point11, point21, int(num_wing_vlm/2+1))), axis=0)
#
#     leading_edge = wing.project(leading_edge_points, direction=np.array([-1., 0., 0.]), plot=do_plots)
#     trailing_edge = wing.project(trailing_edge_points, direction=np.array([1., 0., 0.]), plot=do_plots)
#
#     # Chord Surface
#     chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
#     if mesh_flag_wing:
#         spatial_rep.plot_meshes([chord_surface])
#
#     # upper and lower surface
#     wing_upper_surface_wireframe = wing.project(chord_surface.value + np.array([0., 0., 0.5]), direction=np.array([0., 0., -1.]), grid_search_n=25, plot=do_plots, max_iterations=200)
#     wing_lower_surface_wireframe = wing.project(chord_surface.value - np.array([0., 0., 0.5]), direction=np.array([0., 0., 1.]), grid_search_n=25, plot=do_plots, max_iterations=200)
#
#     # chamber surface
#     wing_camber_surface = am.linspace(wing_upper_surface_wireframe, wing_lower_surface_wireframe, 1)
#     wing_vlm_mesh_name = 'wing_vlm_mesh'
#     sys_rep.add_output(wing_vlm_mesh_name, wing_camber_surface)
#     if mesh_flag_wing:
#         spatial_rep.plot_meshes([wing_camber_surface])
#
#     # OML mesh
#     wing_oml_mesh = am.vstack((wing_upper_surface_wireframe, wing_lower_surface_wireframe))
#     wing_oml_mesh_name = 'wing_oml_mesh'
#     sys_rep.add_output(wing_oml_mesh_name, wing_oml_mesh)
#     if mesh_flag_wing:
#         spatial_rep.plot_meshes([wing_oml_mesh])
#     # endregion
#
#     # region htail mesh
#     plot_tail_mesh = False
#     mesh_htail = False
#     num_htail_vlm = num_spanwise
#     num_chordwise_vlm = num_chordwise
#     point00 = np.array([132.002-10.0, 19.217+4.5, 18.993+3.5]) # * ft2m # Right tip leading edge
#     point01 = np.array([135.993, 19.217, 18.993]) # * ft2m # Right tip trailing edge
#     point10 = np.array([122.905, 0.000, 20.000]) # * ft2m # Center Leading Edge
#     point11 = np.array([134.308, 0.000, 20.000]) # * ft2m # Center Trailing edge
#     point20 = np.array([132.002-10, -19.217-4.5, 18.993+3.5]) # * ft2m # Left tip leading edge
#     point21 = np.array([135.993, -19.217, 18.993]) # * ft2m # Left tip trailing edge
#
#     leading_edge_points = np.linspace(point00, point20, num_htail_vlm)
#     trailing_edge_points = np.linspace(point01, point21, num_htail_vlm)
#
#     leading_edge_htail = htail.project(leading_edge_points, direction=np.array([0., 0., -1.]), plot=plot_tail_mesh)
#     trailing_edge_htail = htail.project(trailing_edge_points, direction=np.array([1., 0., 0.]), plot=plot_tail_mesh)
#
#
#     # Chord Surface
#     htail_chord_surface = am.linspace(leading_edge_htail, trailing_edge_htail, num_chordwise_vlm)
#     if mesh_htail:
#         spatial_rep.plot_meshes([htail_chord_surface])
#
#     # Upper and Lower surface
#     htail_upper_surface_wireframe = htail.project(htail_chord_surface.value + np.array([0., 0., 1.]), direction=np.array([0., 0., -1.]), grid_search_n=25, plot=plot_tail_mesh)
#     htail_lower_surface_wireframe = htail.project(htail_chord_surface.value - np.array([0., 0., 1.]), direction=np.array([0., 0., 1.]), grid_search_n=25, plot=plot_tail_mesh)
#
#     #chamber surface
#     htail_camber_surface = am.linspace(htail_upper_surface_wireframe, htail_lower_surface_wireframe, 1)
#     htail_vlm_mesh_name = 'htail_vlm_mesh'
#     sys_rep.add_output(htail_vlm_mesh_name, htail_camber_surface)
#     if mesh_htail:
#         spatial_rep.plot_meshes([htail_camber_surface])
#
#     # OML mesh
#     htail_oml_mesh = am.vstack((htail_upper_surface_wireframe, htail_lower_surface_wireframe))
#     htail_oml_mesh_name = 'htail_oml_mesh'
#     sys_rep.add_output(htail_oml_mesh_name, htail_oml_mesh)
#     if mesh_htail:
#         spatial_rep.plot_meshes([htail_oml_mesh])
#     # endregion
#
#     # region strut mesh
#
#     plot_strut_mesh = False
#     num_spanwise_strut_vlm = num_spanwise_strut
#     num_chordwise_strut_vlm = num_chordwise_strut
#     vertex00 = np.array([55.573, -12.641, -4.200]) # left leading 1
#     vertex06 = np.array([61.090, -48.994, 5.763]) # left leading 7
#     vertex10 = np.array([57.309, -12.641, -4.200]) # left trailing 1
#     vertex16 = np.array([62.902, -48.994, 5.763]) # left trailing 7
#     vertex20 = np.array([55.573, 12.641, -4.200]) # right leading 1
#     vertex26 = np.array([61.090, 48.994, 5.763]) # right leading 7
#     vertex30 = np.array([57.309, 12.641, -4.200]) # right trailing 1
#     vertex36 = np.array([62.902, 48.994, 5.763]) # right trailing 7
#
#     do_plots_strut_leading = False
#     do_plots_strut_trailing = False
#
#     left_leading_edge = strut.project(np.linspace(vertex00, vertex06, num_spanwise_strut_vlm), direction=np.array([-1., 0., 0.]), plot=do_plots_strut_leading)
#     right_leading_edge = strut.project(np.linspace(vertex20, vertex26, num_spanwise_strut_vlm), direction=np.array([-1., 0., 0.]), plot=do_plots_strut_leading)
#     left_trailing_edge = strut.project(np.linspace(vertex10, vertex16, num_spanwise_strut_vlm) , direction=np.array([1., 0., 0.]), plot=do_plots_strut_trailing)
#     right_trailing_edge = strut.project(np.linspace(vertex30, vertex36, num_spanwise_strut_vlm), direction=np.array([1., 0., 0.]), plot=do_plots_strut_trailing)
#
#     do_strut_plot = False
#
#     # region left strut mesh
#     chord_surface_left = am.linspace(left_leading_edge, left_trailing_edge, num_chordwise_strut_vlm)
#     if plot_strut_mesh:
#         spatial_rep.plot_meshes([chord_surface_left])
#
#     # Upper and Lower surface
#     strut_left_upper_surface_wireframe = strut.project(chord_surface_left.value + np.array([0., 0., 0.5]), direction=np.array([0., 0., -1.]), grid_search_n=25, plot=do_strut_plot, max_iterations=200)
#     strut_left_lower_surface_wireframe = strut.project(chord_surface_left.value - np.array([0., 0., 0.5]), direction=np.array([0., 0., 1.]), grid_search_n=25, plot=do_strut_plot, max_iterations=200)
#
#     # Chamber surface
#     strut_left_camber_surface = am.linspace(strut_left_upper_surface_wireframe, strut_left_lower_surface_wireframe, 1)
#     strut_left_vlm_mesh_name = 'strut_vlm_mesh_left'
#     sys_rep.add_output(strut_left_vlm_mesh_name, strut_left_camber_surface)
#     if plot_strut_mesh:
#         spatial_rep.plot_meshes([strut_left_camber_surface])
#
#     # OML mesh
#     oml_mesh = am.vstack((strut_left_upper_surface_wireframe, strut_left_lower_surface_wireframe))
#     strut_left_oml_mesh_name = 'strut_oml_mesh'
#     sys_rep.add_output(strut_left_oml_mesh_name, oml_mesh)
#     if plot_strut_mesh:
#         spatial_rep.plot_meshes([strut_left_camber_surface])
#     # endregion
#
#     # region right strut mesh
#     chord_surface_right = am.linspace(right_leading_edge, right_trailing_edge, num_chordwise_strut_vlm)
#     if plot_strut_mesh:
#         spatial_rep.plot_meshes([chord_surface_right])
#
#     # Upper and Lower surface
#     strut_right_upper_surface_wireframe = strut.project(chord_surface_right.value + np.array([0., 0., 0.5]), direction=np.array([0., 0., -1.]), grid_search_n=25, plot=do_strut_plot, max_iterations=200)
#     strut_right_lower_surface_wireframe = strut.project(chord_surface_right.value - np.array([0., 0., 0.5]), direction=np.array([0., 0., 1.]), grid_search_n=25, plot=do_strut_plot, max_iterations=200)
#
#     # Chamber surface
#     strut_right_camber_surface = am.linspace(strut_right_upper_surface_wireframe, strut_right_lower_surface_wireframe, 1)
#     strut_right_vlm_mesh_name = 'strut_vlm_mesh_right'
#     sys_rep.add_output(strut_right_vlm_mesh_name, strut_right_camber_surface)
#     if plot_strut_mesh:
#         spatial_rep.plot_meshes([strut_right_camber_surface])
#
#     # OML Mesh
#     strut_oml_mesh = am.vstack((strut_right_upper_surface_wireframe, strut_right_lower_surface_wireframe))
#     strut_right_oml_mesh_name = 'strut_oml_mesh'
#     sys_rep.add_output(strut_right_oml_mesh_name, strut_oml_mesh)
#     if plot_strut_mesh:
#         spatial_rep.plot_meshes([strut_right_camber_surface])
#
#     plot_total_mesh = False
#
#     if plot_total_mesh:
#         spatial_rep.plot_meshes([strut_right_camber_surface, strut_left_camber_surface, wing_camber_surface, htail_camber_surface])
#
#     # endregion
#
#     # endregion
#
#     # endregion
#
#     # region Geometric outputs
#     # Mid chord
#     point20 = np.array([57.575, 49.280, 5.647])  # * ft2m # Leading Edge
#     point21 = np.array([67.088, -49.280, 5.517])  # * ft2m # Trailing edge
#     right_wing_mid_leading_edge = wing.project(point20)
#     right_wing_mid_trailing_edge = wing.project(point21)
#     right_mid_chord = am.norm(right_wing_mid_leading_edge - right_wing_mid_trailing_edge)
#     sys_rep.add_output(name='right_wing_mid_chord', quantity=right_mid_chord)
#
#     # Area
#     wing_area = am.wireframe_area(wireframe=chord_surface)
#     sys_rep.add_output(name='wing_chord_surface_area', quantity=wing_area)
#
#     # endregion
#
#     ft2m = 0.3048
#     # region Sizing
#     tbw_wt = TBWMassProperties()
#     mass, cg, I = tbw_wt.evaluate()
#
#     total_mass_properties = cd.TotalMassPropertiesM3L()
#     total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass, cg, I)
#
#     # endregion
#
#     # region design scenario
#     design_scenario = cd.DesignScenario(name='aircraft_trim')
#
#     # region cruise condtion
#     cruise_model = m3l.Model()
#     cruise_condition = cd.CruiseCondition(name="cruise_1")
#     cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()
#     cruise_condition.set_module_input(name='altitude', val=13106.4)
#     cruise_condition.set_module_input(name='mach_number', val=0.70, dv_flag=False, lower=0.68, upper=0.72)
#     cruise_condition.set_module_input(name='range', val=6482000)
#     cruise_condition.set_module_input(name='pitch_angle', val=np.deg2rad(0), dv_flag=True, lower=0., upper=np.deg2rad(10))
#     cruise_condition.set_module_input(name='flight_path_angle', val=0)
#     cruise_condition.set_module_input(name='roll_angle', val=0)
#     cruise_condition.set_module_input(name='yaw_angle', val=0)
#     cruise_condition.set_module_input(name='wind_angle', val=0)
#     cruise_condition.set_module_input(name='observer_location', val=np.array([0, 0, 500*ft2m]))
#
#     ac_states = cruise_condition.evaluate_ac_states()
#     cruise_model.register_output(ac_states)
#
#     # region Aerodynamics
#     # region VLM
#     vlm_model = VASTFluidSover(
#         surface_names=[
#             wing_vlm_mesh_name,
#             htail_vlm_mesh_name,
#             strut_left_vlm_mesh_name,
#             strut_right_vlm_mesh_name
#         ],
#         surface_shapes=[
#             (1, ) + wing_camber_surface.evaluate().shape[1:],
#             (1, ) + htail_camber_surface.evaluate().shape[1:],
#             (1, ) + strut_left_camber_surface.evaluate().shape[1:],
#             (1, ) + strut_right_camber_surface.evaluate().shape[1:],
#         ],
#         fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
#         mesh_unit='ft',
#         cl0=[0.01,0.,0.,0.]
#     )
#
#     # aero forces and moments
#     vlm_panel_forces, vlm_force, vlm_moment  = vlm_model.evaluate(ac_states=ac_states)
#     cruise_model.register_output(vlm_force)
#     cruise_model.register_output(vlm_moment)
#     # endregion
#
#     # region Viscous drag
#     # wing_ref_area = 1477.109999845  # 1477.109999845 ft^2 = 137.2280094 m^2
#     wing_ref_chord = 110.286  # MAC: 110.286 ft = 33.6151728 m
#     tbw_viscous_drag_model = TbwViscousDragModel(geometry_units='ft')
#     # tbw_viscous_drag_model.set_module_input('area', val=wing_ref_area)
#     tbw_viscous_drag_model.set_module_input('chord', val=wing_ref_chord)
#     tbw_viscous_drag_forces, tbw_viscous_drag_moments = tbw_viscous_drag_model.evaluate(ac_states=ac_states)
#     cruise_model.register_output(tbw_viscous_drag_forces)
#     cruise_model.register_output(tbw_viscous_drag_moments)
#     # endregion
#     # endregion
#
#     # region Propulsion loads
#
#     ref_pt = np.array([0., 0., 2.8])
#     tbw_left_prop_model = tbwPropulsionModel()
#     tbw_left_prop_model.set_module_input('thrust_origin', val=np.array([0., 0., 6.256]))
#     tbw_left_prop_model.set_module_input('ref_pt', val=ref_pt)
#     tbw_left_prop_model.set_module_input('throttle', val = 1., dv_flag=True, lower=0., upper=1.)
#     tbw_left_prop_forces, tbw_left_prop_moments = tbw_left_prop_model.evaluate(ac_states=ac_states)
#     cruise_model.register_output(tbw_left_prop_forces)
#     cruise_model.register_output(tbw_left_prop_moments)
#
#     #region Engine Propulsion
#     # region Left Engine Propulsion
#     # ref_pt = np.array([0., 0., 2.8])
#     # tbw_left_prop_model = cd.tbwPropulsionModel()
#     # tbw_left_prop_model.set_module_input('thrust_origin', val=np.array([61.009, 42.646, 6.256]))
#     # tbw_left_prop_model.set_module_input('ref_pt', val=ref_pt)
#     # tbw_left_prop_model.set_module_input('throttle', val = 1., dv_flag=True, lower=0., upper=1.)
#     # tbw_left_prop_forces, tbw_left_prop_moments = tbw_left_prop_model.evaluate(ac_states=ac_states)
#     # cruise_model.register_output(tbw_left_prop_forces)
#     # cruise_model.register_output(tbw_left_prop_moments)
#     # endregion
#     # region right Engine Propulsion
#     # tbw_right_prop_model = cd.tbwPropulsionModel()
#     # tbw_right_prop_model.set_module_input('thrust_origin', val=np.array([61.009, -42.646, 6.256]))
#     # tbw_right_prop_model.set_module_input('ref_pt', val=ref_pt)
#     # tbw_right_prop_model.set_module_input('throttle_right', val = 1., dv_flag=True, lower=0., upper=1.)
#     # tbw_right_prop_forces, tbw_right_prop_moments = tbw_right_prop_model.evaluate(ac_states=ac_states)
#     # cruise_model.register_output(tbw_right_prop_forces)
#     # cruise_model.register_output(tbw_right_prop_moments)
#     # endregion
#     # endregion
#
#     # endregion
#
#     # region inertial loads
#     inertial_loads_model = cd.InertialLoadsM3L(load_factor=1.)
#     inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg, totoal_mass=total_mass, ac_states=ac_states)
#     cruise_model.register_output(inertial_forces)
#     cruise_model.register_output(inertial_moments)
#     # endregion
#
#     # Total Loads
#     total_forces_moments_model = cd.TotalForcesMomentsM3L()
#     total_forces, total_moments = total_forces_moments_model.evaluate(
#         inertial_forces, inertial_moments,
#         vlm_force, vlm_moment,
#         tbw_left_prop_forces, tbw_left_prop_moments,
#         #tbw_right_prop_forces, tbw_right_prop_moments,
#         tbw_viscous_drag_forces, tbw_viscous_drag_moments
#     )
#     cruise_model.register_output(total_forces)
#     cruise_model.register_output(total_moments)
#
#     # pass total forces/moments + mass properties into EoM model
#     eom_m3l_model = cd.EoMM3LEuler6DOF()
#     trim_residual = eom_m3l_model.evaluate(
#         total_mass=total_mass,
#         total_cg_vector=total_cg,
#         total_inertia_tensor=total_inertia,
#         total_forces=total_forces,
#         total_moments=total_moments,
#         ac_states=ac_states
#     )
#
#     cruise_model.register_output(trim_residual)
#
#     # Add cruise m3l model to cruise condition
#     cruise_condition.add_m3l_model('cruise_model', cruise_model)
#
#     # Add design condition to design scenario
#     design_scenario.add_design_condition(cruise_condition)
#     # endregion
#
#     system_model.add_design_scenario(design_scenario=design_scenario)
#     # endregion
#
#     caddee_csdl_model = caddee.assemble_csdl()
#
#     caddee_csdl_model.create_input(name='h_tail_act', val=np.deg2rad(0.))
#     caddee_csdl_model.add_design_variable(dv_name='h_tail_act', lower=np.deg2rad(-10), upper=np.deg2rad(10), scaler=1.)
#
#     # region Compute total wing CD
#     # wing_vlm_CD = caddee_csdl_model.declare_variable(name='wing_vlm_CD')
#     # caddee_csdl_model.connect(
#     #     f'system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.{wing_vlm_mesh_name}_C_D_total',
#     #     'wing_vlm_CD'
#     # )
#     # wing_Cf = caddee_csdl_model.declare_variable(name='wing_Cf')
#     # caddee_csdl_model.connect(
#     #     'system_model.aircraft_trim.cruise_1.cruise_1.tbw_viscous_drag_model.Cf',
#     #     'wing_Cf'
#     # )
#     # wing_total_CD = wing_vlm_CD + wing_Cf
#     # caddee_csdl_model.register_output(name='wing_total_CD', var=wing_total_CD)
#     # endregion
#
#     caddee_csdl_model.connect('wing_chord_surface_area',
#                               'system_model.aircraft_trim.cruise_1.cruise_1.tbw_viscous_drag_model.area')
#
#     # region Optimization Setup
#     caddee_csdl_model.add_objective('system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual')
#     # caddee_csdl_model.add_constraint(
#     #     name='system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.total_CL',
#     #     equals=0.766)
#     # caddee_csdl_model.add_constraint(name='system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.L_over_D',
#     #                                  equals=25.249,
#     #                                  scaler = 1e-2
#     #                                 )
#     # endregion
#
#     # create and run simulator
#     sim = Simulator(caddee_csdl_model, analytics=True)
#     # VLM - reference point
#     sim['system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.evaluation_pt'] = np.array([0., 0., 2.8])
#
#     sim.run()
#     #sim.compute_total_derivatives()
#     #sim.check_totals()
#
#     prob = CSDLProblem(problem_name='tbw', simulator=sim)
#     optimizer = SLSQP(prob, maxiter=1000, ftol=1E-10)
#     optimizer.solve()
#     optimizer.print_results()
#
#     # region Geometric outputs
#     print('Wing chord surface area (ft^2): ', sim['wing_chord_surface_area'])
#     # endregion
#
#     # region Outputs of CL, CD, L/D
#
#     # region CL
#     CL_total = sim[
#         'system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.total_CL']
#     CL_wing = sim[
#         f'system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.{wing_vlm_mesh_name}_C_L_total']
#     print('Total CL: ', CL_total)
#     print('Wing CL: ', CL_wing)
#     # endregion
#
#     # region VLM CD
#     CD_vlm_total = sim[
#         'system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.total_CD']
#     CD_vlm_wing = sim[
#         f'system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.{wing_vlm_mesh_name}_C_D_total']
#     print('Total VLM CD: ', CD_vlm_total)
#     print('Wing VLM CD: ', CD_vlm_wing)
#     # endregion
#
#     # region Wing Viscous Drag Coefficient
#     Cf = sim['system_model.aircraft_trim.cruise_1.cruise_1.tbw_viscous_drag_model.Cf']
#     print('Cf: ', Cf)
#     # endregion
#
#     # region VLM L/D
#     LoverD_vlm_total = sim[
#         'system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.L_over_D']
#     LoverD_vlm_wing = sim[
#         f'system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.{wing_vlm_mesh_name}_L_over_D']
#     print('Total VLM L/D: ', LoverD_vlm_total)
#     print('Wing VLM L/D: ', LoverD_vlm_wing)
#     # endregion
#     # endregion
#
#     # region Outputs of forces and moments
#
#     print('VLM forces: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.F'])
#     print('Viscous drag forces: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.tbw_viscous_drag_model.F'])
#
#     print('Total forces: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_forces'])
#     print('Total moments:', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_moments'])
#     # endregion
#
#     print('Trim residual: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual'])
#     print('Horizontal tail actuation: ', np.rad2deg(sim['system_parameterization.ffd_set.rotational_section_properties_model.h_tail_act']))
#     print('pitch: ', np.rad2deg(sim['system_model.aircraft_trim.cruise_1.cruise_1.cruise_1_ac_states_operation.cruise_1_pitch_angle']))
#     print('throttle', sim['system_model.aircraft_trim.cruise_1.cruise_1.tbw_prop_model.throttle'])
#
#     return



def trim_at_1g_with_geom_dvs(wing_span_input=170.,  # ft
                             wing_tip_linear_translation_input=0., # ft
                             visualize_flag=False,
                             trim_flag=True
                             ):
    caddee = cd.CADDEE()
    caddee.system_model = system_model = cd.SystemModel()
    caddee.system_representation = sys_rep = cd.SystemRepresentation()
    caddee.system_parameterization = sys_param = cd.SystemParameterization(system_representation=sys_rep)

    # region Geometry
    file_name = 'tbw.stp'

    spatial_rep = sys_rep.spatial_representation
    spatial_rep.import_file(file_name=GEOMETRY_FILES_FOLDER / file_name)
    spatial_rep.refit_geometry(file_name=GEOMETRY_FILES_FOLDER / file_name)

    # region Lifting Surfaces
    # wing
    wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Wing']).keys())
    wing = LiftingSurface(name='wing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)
    if debug_geom_flag:
        wing.plot()
    sys_rep.add_component(wing)

    # Horizontal tail
    tail_primitive_names = list(spatial_rep.get_primitives(search_names=['Htail']).keys())
    htail = cd.LiftingSurface(name='h_tail', spatial_representation=spatial_rep,
                              primitive_names=tail_primitive_names)
    if debug_geom_flag:
        htail.plot()
    sys_rep.add_component(htail)

    # Strut
    strut_primitive_names = list(spatial_rep.get_primitives(search_names=['Strut']).keys())
    strut = cd.LiftingSurface(name='strut', spatial_representation=spatial_rep,
                              primitive_names=strut_primitive_names)
    if debug_geom_flag:
        strut.plot()
    sys_rep.add_component(strut)

    # jury
    jury_primitive_names = list(spatial_rep.get_primitives(search_names=['Jury']).keys())
    jury = cd.LiftingSurface(name='jury', spatial_representation=spatial_rep, primitive_names=jury_primitive_names)
    if debug_geom_flag:
        jury.plot()
    sys_rep.add_component(jury)
    # endregion
    # endregion

    # region Actuations
    # Tail FFD
    htail_geometry_primitives = htail.get_geometry_primitives()
    htail_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(
        htail_geometry_primitives,
        num_control_points=(11, 2, 2), order=(4, 2, 2),
        xyz_to_uvw_indices=(1, 0, 2))
    htail_ffd_block = cd.SRBGFFDBlock(name='htail_ffd_block',
                                      primitive=htail_ffd_bspline_volume,
                                      embedded_entities=htail_geometry_primitives)
    htail_ffd_block.add_scale_v(
        name='htail_linear_taper',
        order=2, num_dof=3, value=np.array([0., 0., 0.]),
        cost_factor=1.)
    htail_ffd_block.add_rotation_u(name='htail_twist_distribution',
                                   connection_name='h_tail_act', order=1,
                                   num_dof=1, value=np.array([np.deg2rad(1.75)]))
    # ffd_blocks = {
    #     htail_ffd_block.name: htail_ffd_block
    # }
    # ffd_set = cd.SRBGFFDSet(name='ffd_set', ffd_blocks=ffd_blocks)
    # sys_param.add_geometry_parameterization(ffd_set)
    # endregion

    # region FFD
    surfaces = []
    surfaces.append('Wing')
    surfaces.append('Jury')
    surfaces.append('Strut')
    surfaces.append('Gear Pod')
    ffd_primitive_names = list(spatial_rep.get_primitives(search_names=surfaces).keys())
    ffd_components = LiftingSurface(name='FFD Components', spatial_representation=spatial_rep,
                                    primitive_names=ffd_primitive_names)
    if debug_geom_flag:
        ffd_components.plot()

    flag_blspline_volume = False
    flag_block = False

    wing_geometry_primitives = ffd_components.get_geometry_primitives()

    points = np.array([
        [
            [
                [68.136 - 21.0, 85.291 + 0.5, 4.741 - 9.5],  # right tip chord
                [68.136 - 21.0, 85.291 + 0.5, 4.741 + 4.6]
            ],
            [
                [71.664 + 2.0, 85.291 + 0.5, 4.741 - 9.5],
                [71.664 + 2.0, 85.291 + 0.5, 4.741 + 4.6]
            ]
        ],
        [
            [
                [68.136 - 21.0, -85.291 - 0.5, 4.741 - 9.5],  # left tip chord
                [68.136 - 21.0, -85.291 - 0.5, 4.741 + 4.6]
            ],
            [
                [71.664 + 1.0, -85.291 - 0.5, 4.741 - 9.5],
                [71.664 + 1.0, -85.291 - 0.5, 4.741 + 4.6]
            ]
        ]
    ])

    wing_strut_jury_ffd_bspline_volume = create_bspline_from_corners(points, order=(4, 2, 2),
                                                                     num_control_points=(11, 2, 2))
    if flag_blspline_volume:
        wing_strut_jury_ffd_bspline_volume.plot()
    wing_strut_jury_ffd_block = cd.SRBGFFDBlock(name='wing_strut_jury_ffd_block',
                                                primitive=wing_strut_jury_ffd_bspline_volume,
                                                embedded_entities=wing_geometry_primitives)
    if flag_block:
        wing_strut_jury_ffd_block.plot()

    wing_strut_jury_ffd_block.add_translation_u(name='wing_strut_jury_linear_span',
                                                order=2, num_dof=2,
                                                cost_factor=1.)
    # wing_strut_jury_ffd_block.add_scale_v(name='wing_strut_jury_linear_taper',
    #                                       order=2, num_dof=2,
    #                                       cost_factor=1.)

    # Sweep
    wing_strut_jury_ffd_block.add_translation_v(name='wing_strut_jury_linear_sweep',
                                                order=2, num_dof=3, connection_name='wing_strut_jury_translation',
                                                value=np.array([0., 0., 0.]),
                                                cost_factor=1.)  # todo: Create an input; make DV; connect to this parameter
    wing_strut_jury_ffd_block.add_rotation_u(name='wing_strut_jury_twist_distribution',
                                             connection_name='wing_strut_jury_twist', order=2, value=np.zeros(5),
                                             num_dof=5)  # todo: Create an input; make DV; connect to this parameter

    ffd_blocks = {
        wing_strut_jury_ffd_block.name: wing_strut_jury_ffd_block,
        htail_ffd_block.name: htail_ffd_block
    }
    ffd_set = cd.SRBGFFDSet(name='ffd_set', ffd_blocks=ffd_blocks)
    sys_param.add_geometry_parameterization(ffd_set)

    # Setting a value for span scale (in %)
    point20 = np.array([68.136 - 21.0, 85.291 + 0.5, 4.741 + 4.6])
    point21 = np.array([68.136 - 21.0, -85.291 - 0.5, 4.741 + 4.6])
    span_left_tip_leading_edge = ffd_components.project(point20, force_reprojection=True)
    span_right_tip_leading_edge = ffd_components.project(point21, force_reprojection=True)
    span_tip_leading_edge = am.norm(span_left_tip_leading_edge - span_right_tip_leading_edge)
    sys_param.add_input(name='wing_span', quantity=span_tip_leading_edge)
    # endregion

    sys_param.setup()

    # region meshes

    num_spanwise_wing = 31
    num_chordwise_wing = 5

    num_spanwise_strut = 21
    num_chordwise_strut = 5

    num_spanwise_htail = 21
    num_chordwise_htail = 5

    # region wing mesh
    mesh_flag_wing = False
    point00 = np.array([68.035, 85.291, 4.704 + 0.1])  # * ft2m # Right tip leading edge
    point01 = np.array([71.790, 85.291, 4.708 + 0.1])  # * ft2m # Right tip trailing edge
    point10 = np.array([47.231, 0.000, 6.937 + 0.1])  # * ft2m # Center Leading Edge
    point11 = np.array([57.953, 0.000, 6.574 + 0.1])  # * ft2m # Center Trailing edge
    point20 = np.array([68.035, -85.291, 4.704 + 0.1])  # * ft2m # Left tip leading edge
    point21 = np.array([71.790, -85.291, 4.708 + 0.1])  # * ft2m # Left tip trailing edge

    do_plots = False

    leading_edge_points = np.concatenate((np.linspace(point00, point10, int(num_spanwise_wing / 2 + 1))[0:-1, :],
                                          np.linspace(point10, point20, int(num_spanwise_wing / 2 + 1))), axis=0)
    trailing_edge_points = np.concatenate((np.linspace(point01, point11, int(num_spanwise_wing / 2 + 1))[0:-1, :],
                                           np.linspace(point11, point21, int(num_spanwise_wing / 2 + 1))), axis=0)

    leading_edge = wing.project(leading_edge_points, direction=np.array([-1., 0., 0.]), plot=do_plots)
    trailing_edge = wing.project(trailing_edge_points, direction=np.array([1., 0., 0.]), plot=do_plots)

    # Chord Surface
    chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_wing)
    if mesh_flag_wing:
        spatial_rep.plot_meshes([chord_surface])

    # upper and lower surface
    wing_upper_surface_wireframe = wing.project(chord_surface.value + np.array([0., 0., 0.5]),
                                                direction=np.array([0., 0., -1.]), grid_search_n=25, plot=do_plots,
                                                max_iterations=200)
    wing_lower_surface_wireframe = wing.project(chord_surface.value - np.array([0., 0., 0.5]),
                                                direction=np.array([0., 0., 1.]), grid_search_n=25, plot=do_plots,
                                                max_iterations=200)

    # chamber surface
    wing_camber_surface = am.linspace(wing_upper_surface_wireframe, wing_lower_surface_wireframe, 1)
    wing_vlm_mesh_name = 'wing_vlm_mesh'
    sys_rep.add_output(wing_vlm_mesh_name, wing_camber_surface)
    if mesh_flag_wing:
        spatial_rep.plot_meshes([wing_camber_surface])

    # OML mesh
    wing_oml_mesh = am.vstack((wing_upper_surface_wireframe, wing_lower_surface_wireframe))
    wing_oml_mesh_name = 'wing_oml_mesh'
    sys_rep.add_output(wing_oml_mesh_name, wing_oml_mesh)
    if mesh_flag_wing:
        spatial_rep.plot_meshes([wing_oml_mesh])
    # endregion

    # region htail mesh
    plot_tail_mesh = False
    mesh_htail = False

    point00 = np.array([132.002 - 10.0, 19.217 + 4.5, 18.993 + 3.5])  # * ft2m # Right tip leading edge
    point01 = np.array([135.993, 19.217, 18.993])  # * ft2m # Right tip trailing edge
    point10 = np.array([122.905, 0.000, 20.000])  # * ft2m # Center Leading Edge
    point11 = np.array([134.308, 0.000, 20.000])  # * ft2m # Center Trailing edge
    point20 = np.array([132.002 - 10, -19.217 - 4.5, 18.993 + 3.5])  # * ft2m # Left tip leading edge
    point21 = np.array([135.993, -19.217, 18.993])  # * ft2m # Left tip trailing edge

    leading_edge_points = np.linspace(point00, point20, num_spanwise_htail)
    trailing_edge_points = np.linspace(point01, point21, num_spanwise_htail)

    leading_edge_htail = htail.project(leading_edge_points, direction=np.array([0., 0., -1.]), plot=plot_tail_mesh)
    trailing_edge_htail = htail.project(trailing_edge_points, direction=np.array([1., 0., 0.]), plot=plot_tail_mesh)

    # Chord Surface
    htail_chord_surface = am.linspace(leading_edge_htail, trailing_edge_htail, num_chordwise_htail)
    if mesh_htail:
        spatial_rep.plot_meshes([htail_chord_surface])

    # Upper and Lower surface
    htail_upper_surface_wireframe = htail.project(htail_chord_surface.value + np.array([0., 0., 1.]),
                                                  direction=np.array([0., 0., -1.]), grid_search_n=25,
                                                  plot=plot_tail_mesh)
    htail_lower_surface_wireframe = htail.project(htail_chord_surface.value - np.array([0., 0., 1.]),
                                                  direction=np.array([0., 0., 1.]), grid_search_n=25,
                                                  plot=plot_tail_mesh)

    # chamber surface
    htail_camber_surface = am.linspace(htail_upper_surface_wireframe, htail_lower_surface_wireframe, 1)
    htail_vlm_mesh_name = 'htail_vlm_mesh'
    sys_rep.add_output(htail_vlm_mesh_name, htail_camber_surface)
    if mesh_htail:
        spatial_rep.plot_meshes([htail_camber_surface])

    # OML mesh
    htail_oml_mesh = am.vstack((htail_upper_surface_wireframe, htail_lower_surface_wireframe))
    htail_oml_mesh_name = 'htail_oml_mesh'
    sys_rep.add_output(htail_oml_mesh_name, htail_oml_mesh)
    if mesh_htail:
        spatial_rep.plot_meshes([htail_oml_mesh])
    # endregion

    # region strut mesh

    plot_strut_mesh = False
    vertex00 = np.array([55.573, -12.641, -4.200])  # left leading 1
    vertex06 = np.array([61.090, -48.994, 5.763])  # left leading 7
    vertex10 = np.array([57.309, -12.641, -4.200])  # left trailing 1
    vertex16 = np.array([62.902, -48.994, 5.763])  # left trailing 7
    vertex20 = np.array([55.573, 12.641, -4.200])  # right leading 1
    vertex26 = np.array([61.090, 48.994, 5.763])  # right leading 7
    vertex30 = np.array([57.309, 12.641, -4.200])  # right trailing 1
    vertex36 = np.array([62.902, 48.994, 5.763])  # right trailing 7

    do_plots_strut_leading = False
    do_plots_strut_trailing = False

    left_leading_edge = strut.project(np.linspace(vertex00, vertex06, num_spanwise_strut),
                                      direction=np.array([-1., 0., 0.]), plot=do_plots_strut_leading)
    right_leading_edge = strut.project(np.linspace(vertex20, vertex26, num_spanwise_strut),
                                       direction=np.array([-1., 0., 0.]), plot=do_plots_strut_leading)
    left_trailing_edge = strut.project(np.linspace(vertex10, vertex16, num_spanwise_strut),
                                       direction=np.array([1., 0., 0.]), plot=do_plots_strut_trailing)
    right_trailing_edge = strut.project(np.linspace(vertex30, vertex36, num_spanwise_strut),
                                        direction=np.array([1., 0., 0.]), plot=do_plots_strut_trailing)

    do_strut_plot = False

    # region left strut mesh
    chord_surface_left = am.linspace(left_leading_edge, left_trailing_edge, num_chordwise_strut)
    if plot_strut_mesh:
        spatial_rep.plot_meshes([chord_surface_left])

    # Upper and Lower surface
    strut_left_upper_surface_wireframe = strut.project(chord_surface_left.value + np.array([0., 0., 0.5]),
                                                       direction=np.array([0., 0., -1.]), grid_search_n=25,
                                                       plot=do_strut_plot, max_iterations=200)
    strut_left_lower_surface_wireframe = strut.project(chord_surface_left.value - np.array([0., 0., 0.5]),
                                                       direction=np.array([0., 0., 1.]), grid_search_n=25,
                                                       plot=do_strut_plot, max_iterations=200)

    # Chamber surface
    strut_left_camber_surface = am.linspace(strut_left_upper_surface_wireframe, strut_left_lower_surface_wireframe,
                                            1)
    strut_left_vlm_mesh_name = 'strut_vlm_mesh_left'
    sys_rep.add_output(strut_left_vlm_mesh_name, strut_left_camber_surface)
    if plot_strut_mesh:
        spatial_rep.plot_meshes([strut_left_camber_surface])

    # OML mesh
    oml_mesh = am.vstack((strut_left_upper_surface_wireframe, strut_left_lower_surface_wireframe))
    strut_left_oml_mesh_name = 'strut_oml_mesh'
    sys_rep.add_output(strut_left_oml_mesh_name, oml_mesh)
    if plot_strut_mesh:
        spatial_rep.plot_meshes([strut_left_camber_surface])
    # endregion

    # region right strut mesh
    chord_surface_right = am.linspace(right_leading_edge, right_trailing_edge, num_chordwise_strut)
    if plot_strut_mesh:
        spatial_rep.plot_meshes([chord_surface_right])

    # Upper and Lower surface
    strut_right_upper_surface_wireframe = strut.project(chord_surface_right.value + np.array([0., 0., 0.5]),
                                                        direction=np.array([0., 0., -1.]), grid_search_n=25,
                                                        plot=do_strut_plot, max_iterations=200)
    strut_right_lower_surface_wireframe = strut.project(chord_surface_right.value - np.array([0., 0., 0.5]),
                                                        direction=np.array([0., 0., 1.]), grid_search_n=25,
                                                        plot=do_strut_plot, max_iterations=200)

    # Chamber surface
    strut_right_camber_surface = am.linspace(strut_right_upper_surface_wireframe,
                                             strut_right_lower_surface_wireframe, 1)
    strut_right_vlm_mesh_name = 'strut_vlm_mesh_right'
    sys_rep.add_output(strut_right_vlm_mesh_name, strut_right_camber_surface)
    if plot_strut_mesh:
        spatial_rep.plot_meshes([strut_right_camber_surface])

    # OML Mesh
    strut_oml_mesh = am.vstack((strut_right_upper_surface_wireframe, strut_right_lower_surface_wireframe))
    strut_right_oml_mesh_name = 'strut_oml_mesh'
    sys_rep.add_output(strut_right_oml_mesh_name, strut_oml_mesh)
    if plot_strut_mesh:
        spatial_rep.plot_meshes([strut_right_camber_surface])

    plot_total_mesh = False

    if plot_total_mesh:
        spatial_rep.plot_meshes(
            [strut_right_camber_surface, strut_left_camber_surface, wing_camber_surface, htail_camber_surface])

    # endregion

    # endregion

    # endregion

    # region Geometric outputs
    # Mid chord
    point20 = np.array([57.575, 49.280, 5.647])  # * ft2m # Leading Edge
    point21 = np.array([67.088, -49.280, 5.517])  # * ft2m # Trailing edge
    right_wing_mid_leading_edge = wing.project(point20)
    right_wing_mid_trailing_edge = wing.project(point21)
    right_mid_chord = am.norm(right_wing_mid_leading_edge - right_wing_mid_trailing_edge)
    sys_rep.add_output(name='right_wing_mid_chord', quantity=right_mid_chord)

    # Area
    wing_area = am.wireframe_area(wireframe=chord_surface)
    sys_rep.add_output(name='wing_chord_surface_area', quantity=wing_area)

    # endregion

    ft2m = 0.3048
    # region Sizing
    tbw_wt = TBWMassProperties()
    mass, cg, I = tbw_wt.evaluate()

    total_mass_properties = cd.TotalMassPropertiesM3L()
    total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass, cg, I)

    # endregion

    # region design scenario
    design_scenario = cd.DesignScenario(name='aircraft_trim')

    # region cruise condtion
    cruise_model = m3l.Model()
    cruise_condition = cd.CruiseCondition(name="cruise_1")
    cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()
    cruise_condition.set_module_input(name='altitude', val=13106.4)
    cruise_condition.set_module_input(name='mach_number', val=0.70, dv_flag=False, lower=0.68, upper=0.72)
    cruise_condition.set_module_input(name='range', val=6482000)
    cruise_condition.set_module_input(name='pitch_angle', val=np.deg2rad(0), dv_flag=True, lower=0.,
                                      upper=np.deg2rad(10))
    cruise_condition.set_module_input(name='flight_path_angle', val=0)
    cruise_condition.set_module_input(name='roll_angle', val=0)
    cruise_condition.set_module_input(name='yaw_angle', val=0)
    cruise_condition.set_module_input(name='wind_angle', val=0)
    cruise_condition.set_module_input(name='observer_location', val=np.array([0, 0, 500 * ft2m]))

    ac_states = cruise_condition.evaluate_ac_states()
    cruise_model.register_output(ac_states)

    # region Aerodynamics
    # region VLM
    vlm_model = VASTFluidSover(
        surface_names=[
            wing_vlm_mesh_name,
            htail_vlm_mesh_name,
            strut_left_vlm_mesh_name,
            strut_right_vlm_mesh_name
        ],
        surface_shapes=[
            (1,) + wing_camber_surface.evaluate().shape[1:],
            (1,) + htail_camber_surface.evaluate().shape[1:],
            (1,) + strut_left_camber_surface.evaluate().shape[1:],
            (1,) + strut_right_camber_surface.evaluate().shape[1:],
        ],
        fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
        mesh_unit='ft',
        cl0=[0.01, 0., 0., 0.]
    )

    # aero forces and moments
    vlm_panel_forces, vlm_force, vlm_moment = vlm_model.evaluate(ac_states=ac_states)
    cruise_model.register_output(vlm_force)
    cruise_model.register_output(vlm_moment)
    # endregion

    # region Viscous drag
    # wing_ref_area = 1477.109999845  # 1477.109999845 ft^2 = 137.2280094 m^2
    wing_ref_chord = 110.286  # MAC: 110.286 ft = 33.6151728 m
    tbw_viscous_drag_model = TbwViscousDragModel(geometry_units='ft')
    # tbw_viscous_drag_model.set_module_input('area', val=wing_ref_area)
    tbw_viscous_drag_model.set_module_input('chord', val=wing_ref_chord)
    tbw_viscous_drag_forces, tbw_viscous_drag_moments = tbw_viscous_drag_model.evaluate(ac_states=ac_states)
    cruise_model.register_output(tbw_viscous_drag_forces)
    cruise_model.register_output(tbw_viscous_drag_moments)
    # endregion
    # endregion

    # region Propulsion loads

    ref_pt = np.array([0., 0., 2.8])
    tbw_left_prop_model = tbwPropulsionModel()
    tbw_left_prop_model.set_module_input('thrust_origin', val=np.array([0., 0., 6.256]))
    tbw_left_prop_model.set_module_input('ref_pt', val=ref_pt)
    tbw_left_prop_model.set_module_input('throttle', val=1., dv_flag=True, lower=0., upper=1.)
    tbw_left_prop_forces, tbw_left_prop_moments = tbw_left_prop_model.evaluate(ac_states=ac_states)
    cruise_model.register_output(tbw_left_prop_forces)
    cruise_model.register_output(tbw_left_prop_moments)

    # region Engine Propulsion
    # region Left Engine Propulsion
    # ref_pt = np.array([0., 0., 2.8])
    # tbw_left_prop_model = cd.tbwPropulsionModel()
    # tbw_left_prop_model.set_module_input('thrust_origin', val=np.array([61.009, 42.646, 6.256]))
    # tbw_left_prop_model.set_module_input('ref_pt', val=ref_pt)
    # tbw_left_prop_model.set_module_input('throttle', val = 1., dv_flag=True, lower=0., upper=1.)
    # tbw_left_prop_forces, tbw_left_prop_moments = tbw_left_prop_model.evaluate(ac_states=ac_states)
    # cruise_model.register_output(tbw_left_prop_forces)
    # cruise_model.register_output(tbw_left_prop_moments)
    # endregion
    # region right Engine Propulsion
    # tbw_right_prop_model = cd.tbwPropulsionModel()
    # tbw_right_prop_model.set_module_input('thrust_origin', val=np.array([61.009, -42.646, 6.256]))
    # tbw_right_prop_model.set_module_input('ref_pt', val=ref_pt)
    # tbw_right_prop_model.set_module_input('throttle_right', val = 1., dv_flag=True, lower=0., upper=1.)
    # tbw_right_prop_forces, tbw_right_prop_moments = tbw_right_prop_model.evaluate(ac_states=ac_states)
    # cruise_model.register_output(tbw_right_prop_forces)
    # cruise_model.register_output(tbw_right_prop_moments)
    # endregion
    # endregion

    # endregion

    # region inertial loads
    inertial_loads_model = cd.InertialLoadsM3L(load_factor=1.)
    inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg,
                                                                      totoal_mass=total_mass, ac_states=ac_states)
    cruise_model.register_output(inertial_forces)
    cruise_model.register_output(inertial_moments)
    # endregion

    # Total Loads
    total_forces_moments_model = cd.TotalForcesMomentsM3L()
    total_forces, total_moments = total_forces_moments_model.evaluate(
        inertial_forces, inertial_moments,
        vlm_force, vlm_moment,
        tbw_left_prop_forces, tbw_left_prop_moments,
        # tbw_right_prop_forces, tbw_right_prop_moments,
        tbw_viscous_drag_forces, tbw_viscous_drag_moments
    )
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
    # endregion

    caddee_csdl_model = caddee.assemble_csdl()

    # region FFD variables

    # Wing span
    # caddee_csdl_model.create_input(name='wing_span', val=160.08993301)
    caddee_csdl_model.create_input(name='wing_span', val=wing_span_input)
    if not trim_flag:
        caddee_csdl_model.add_design_variable(dv_name='wing_span', lower=140., upper=200., scaler=1e-2)

    # Wing sweep
    wing_tip_translation = caddee_csdl_model.create_input(name='wing_tip_linear_translation',
                                                          val=wing_tip_linear_translation_input)
    if not trim_flag:
        caddee_csdl_model.add_design_variable(dv_name='wing_tip_linear_translation', lower=-10., upper=10., scaler=1e-1)

    ffd_block_translations_for_sweep = caddee_csdl_model.create_output(name='wing_strut_jury_translation',
                                                                       shape=(3,), val=0.)
    ffd_block_translations_for_sweep[0] = wing_tip_translation * 1.
    ffd_block_translations_for_sweep[2] = wing_tip_translation * 1.

    # Horizontal tail actuation
    caddee_csdl_model.create_input(name='h_tail_act', val=np.deg2rad(0.))
    if trim_flag:
        caddee_csdl_model.add_design_variable(dv_name='h_tail_act', lower=np.deg2rad(-10), upper=np.deg2rad(10),
                                              scaler=1.)
    # endregion

    # region Compute total wing drag
    wing_drag_vlm = caddee_csdl_model.declare_variable(name='wing_drag_vlm')
    caddee_csdl_model.connect(
        'system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.wing_vlm_mesh_total_drag',
        'wing_drag_vlm')
    wing_drag_viscous = caddee_csdl_model.declare_variable(name='wing_drag_viscous')
    caddee_csdl_model.connect(
        'system_model.aircraft_trim.cruise_1.cruise_1.tbw_viscous_drag_model.D',
        'wing_drag_viscous'
    )
    wing_drag_total = wing_drag_viscous + wing_drag_vlm
    caddee_csdl_model.register_output(name='wing_drag_total', var=wing_drag_total)
    # endregion

    # region Compute total wing CD
    wing_cd_vlm = caddee_csdl_model.declare_variable(name='wing_cd_vlm')
    caddee_csdl_model.connect(
        f'system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.{wing_vlm_mesh_name}_C_D_total',
        'wing_cd_vlm'
    )
    wing_cf_viscous = caddee_csdl_model.declare_variable(name='wing_cf_viscous')
    caddee_csdl_model.connect(
        'system_model.aircraft_trim.cruise_1.cruise_1.tbw_viscous_drag_model.Cf',
        'wing_cf_viscous'
    )

    wing_cd_total = wing_cd_vlm + wing_cf_viscous
    caddee_csdl_model.register_output(name='wing_cd_total', var=wing_cd_total)
    # endregion

    caddee_csdl_model.connect('wing_chord_surface_area',
                              'system_model.aircraft_trim.cruise_1.cruise_1.tbw_viscous_drag_model.area')

    # region Optimization Setup
    caddee_csdl_model.add_objective(
        'system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual')
    # caddee_csdl_model.add_constraint(
    #     name='system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.total_CL',
    #     equals=0.766)
    # caddee_csdl_model.add_constraint(name='system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.L_over_D',
    #                                  equals=25.249,
    #                                  scaler = 1e-2
    #                                 )
    # endregion

    # create and run simulator
    sim = Simulator(caddee_csdl_model, analytics=True)
    # VLM - reference point
    sim[
        'system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.evaluation_pt'] = np.array(
        [0., 0., 2.8])

    sim.run()
    # sim.compute_total_derivatives()
    # sim.check_totals()

    if visualize_flag:
        geom = sim['design_geometry']
        spatial_rep.update(geom)
        strut_right_camber_surface.evaluate(input=geom)
        strut_left_camber_surface.evaluate(input=geom)
        wing_camber_surface.evaluate(input=geom)
        htail_camber_surface.evaluate(input=geom)
        spatial_rep.plot_meshes(
            [strut_right_camber_surface, strut_left_camber_surface, wing_camber_surface, htail_camber_surface])

    if trim_flag:
        prob = CSDLProblem(problem_name='tbw_shape_opt', simulator=sim)
        optimizer = SLSQP(prob, maxiter=1000, ftol=1E-10)
        optimizer.solve()
        optimizer.print_results()
        assert optimizer.scipy_output.success
        assert sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual'] < 1e-2

    # region Geometric outputs
    print('Wing chord surface area (ft^2): ', sim['wing_chord_surface_area'])
    print('Wing span (ft): ', sim['wing_span'])
    # endregion

    # region Outputs of CL, CD, L/D

    # region CL
    CL_total = sim[
        'system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.total_CL']
    CL_wing = sim[
        f'system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.{wing_vlm_mesh_name}_C_L_total']
    print('Total CL: ', CL_total)
    print('Wing CL: ', CL_wing)
    # endregion

    # region VLM CD
    CD_vlm_total = sim[
        'system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.total_CD']
    CD_vlm_wing = sim['wing_cd_vlm']
    print('Total VLM CD: ', CD_vlm_total)
    print('Wing VLM CD: ', CD_vlm_wing)
    # endregion

    # region Wing Viscous Drag Coefficient
    Cf = sim['wing_cf_viscous']
    print('Cf: ', Cf)
    # endregion

    # region Total wing CD
    total_wing_cd = sim['wing_cd_total']
    print('Total wing CD: ', total_wing_cd)
    # endregion

    # region VLM L/D
    LoverD_vlm_total = sim[
        'system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.L_over_D']
    LoverD_vlm_wing = sim[
        f'system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.{wing_vlm_mesh_name}_L_over_D']
    print('Total VLM L/D: ', LoverD_vlm_total)
    print('Wing VLM L/D: ', LoverD_vlm_wing)
    # endregion
    # endregion

    # region Outputs of forces and moments

    print('Wing VLM drag: ', sim[
        'system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.wing_vlm_mesh_total_drag'])
    print('Wing viscous drag: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.tbw_viscous_drag_model.D'])
    print('Wing total drag: ', sim['wing_drag_total'])

    print('VLM forces: ', sim[
        'system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.F'])
    print('Viscous drag forces: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.tbw_viscous_drag_model.F'])

    print('Total forces: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_forces'])
    print('Total moments:', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_moments'])
    # endregion

    print('Trim residual: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual'])
    print('Horizontal tail actuation: ',
          np.rad2deg(sim['system_parameterization.ffd_set.rotational_section_properties_model.h_tail_act']))
    print('pitch: ', np.rad2deg(
        sim['system_model.aircraft_trim.cruise_1.cruise_1.cruise_1_ac_states_operation.cruise_1_pitch_angle']))
    print('throttle', sim['system_model.aircraft_trim.cruise_1.cruise_1.tbw_prop_model.throttle'])


    print('CDi: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.wing_vlm_mesh_C_D_i'])
    # region Output
    output_dict = {
        'wing_span': sim['wing_span'][0],
        'wing_tip_translation': sim['wing_tip_linear_translation'][0],
        'wing_area': sim['wing_chord_surface_area'][0],
        'wing_vlm_CL': CL_wing,
        'wing_vlm_CD': CD_vlm_wing[0][0],
        'wing_Cf': Cf,
        'wing_CD_total': total_wing_cd[0],
        'wing_vlm_drag': sim['system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.wing_vlm_mesh_total_drag'][0][0],
        'wing_viscous_drag': sim['system_model.aircraft_trim.cruise_1.cruise_1.tbw_viscous_drag_model.D'][0][0],
        'wing_total_drag': sim['wing_drag_total'][0]
    }
    # endregion

    return output_dict


def shape_vars_sweeps_at_1g():

    # region Sweep over wing span
    span_list = np.linspace(140, 200, num=7)
    # span_list = [170, ]

    span_sweep_span_output = np.empty(len(span_list))
    span_sweep_tip_translation_output = np.empty(len(span_list))
    span_sweep_area_output = np.empty(len(span_list))
    span_sweep_vlm_cl_output = np.empty(len(span_list))
    span_sweep_vlm_cd_output = np.empty(len(span_list))
    span_sweep_cf_output = np.empty(len(span_list))
    span_sweep_total_cd_output = np.empty(len(span_list))
    span_sweep_vlm_drag_output = np.empty(len(span_list))
    span_sweep_viscous_drag_output = np.empty(len(span_list))
    span_sweep_total_drag_output = np.empty(len(span_list))

    for idx, span in enumerate(span_list):
        output_dict = trim_at_1g_with_geom_dvs(wing_span_input=span,
                                               trim_flag=True,
                                               visualize_flag=False)
        print(output_dict)
        span_sweep_span_output[idx] = output_dict['wing_span']
        span_sweep_tip_translation_output[idx] = output_dict['wing_tip_translation']
        span_sweep_area_output[idx] = output_dict['wing_area']
        span_sweep_vlm_cl_output[idx] = output_dict['wing_vlm_CL']
        span_sweep_vlm_cd_output[idx] = output_dict['wing_vlm_CD']
        span_sweep_cf_output[idx] = output_dict['wing_Cf']
        span_sweep_total_cd_output[idx] = output_dict['wing_CD_total']
        span_sweep_vlm_drag_output[idx] = output_dict['wing_vlm_drag']
        span_sweep_viscous_drag_output[idx] = output_dict['wing_viscous_drag']
        span_sweep_total_drag_output[idx] = output_dict['wing_total_drag']
        # input("Press Enter to continue...")

    span_sweep_df = pd.DataFrame(
        data={
            'Span (ft)': span_sweep_span_output,
            'Tip translation (ft)': span_sweep_tip_translation_output,
            'Area (ft^2)': span_sweep_area_output,
            'VLM CL': span_sweep_vlm_cl_output,
            'VLM CD': span_sweep_vlm_cd_output,
            'Cf': span_sweep_cf_output,
            'Total CD': span_sweep_total_cd_output,
            'VLM drag (N)': span_sweep_vlm_drag_output,
            'Viscous drag (N)': span_sweep_viscous_drag_output,
            'Total drag (N)': span_sweep_total_drag_output,
        }
    )
    span_sweep_df.to_excel(f'TbwGeomDvSweeps_Span.xlsx')
    print(span_sweep_df)
    # endregion

    # # region Sweep over wing sweep
    # # tip_translation_list = np.linspace(-30, 30, num=7)
    # tip_translation_list = [0., ]
    #
    # sweep_sweep_span_output = np.empty(len(tip_translation_list))
    # sweep_sweep_tip_translation_output = np.empty(len(tip_translation_list))
    # sweep_sweep_area_output = np.empty(len(tip_translation_list))
    # sweep_sweep_vlm_cl_output = np.empty(len(tip_translation_list))
    # sweep_sweep_vlm_cd_output = np.empty(len(tip_translation_list))
    # sweep_sweep_cf_output = np.empty(len(tip_translation_list))
    # sweep_sweep_total_cd_output = np.empty(len(tip_translation_list))
    # sweep_sweep_vlm_drag_output = np.empty(len(tip_translation_list))
    # sweep_sweep_viscous_drag_output = np.empty(len(tip_translation_list))
    # sweep_sweep_total_drag_output = np.empty(len(tip_translation_list))
    #
    # for idx, tip_translation in enumerate(tip_translation_list):
    #     output_dict = trim_at_1g_with_geom_dvs(wing_tip_linear_translation_input=tip_translation,
    #                                            trim_flag=True,
    #                                            visualize_flag=True)
    #     print(output_dict)
    #     sweep_sweep_span_output[idx] = output_dict['wing_span']
    #     sweep_sweep_tip_translation_output[idx] = output_dict['wing_tip_translation']
    #     sweep_sweep_area_output[idx] = output_dict['wing_area']
    #     sweep_sweep_vlm_cl_output[idx] = output_dict['wing_vlm_CL']
    #     sweep_sweep_vlm_cd_output[idx] = output_dict['wing_vlm_CD']
    #     sweep_sweep_cf_output[idx] = output_dict['wing_Cf']
    #     sweep_sweep_total_cd_output[idx] = output_dict['wing_CD_total']
    #     sweep_sweep_vlm_drag_output[idx] = output_dict['wing_vlm_drag']
    #     sweep_sweep_viscous_drag_output[idx] = output_dict['wing_viscous_drag']
    #     sweep_sweep_total_drag_output[idx] = output_dict['wing_total_drag']
    #     input("Press Enter to continue...")
    #
    # sweep_sweep_df = pd.DataFrame(
    #     data={
    #         'Span (ft)': span_sweep_span_output,
    #         'Tip translation (ft)': sweep_sweep_tip_translation_output,
    #         'Area (ft^2)': sweep_sweep_area_output,
    #         'VLM CL': sweep_sweep_vlm_cl_output,
    #         'VLM CD': sweep_sweep_vlm_cd_output,
    #         'Cf': sweep_sweep_cf_output,
    #         'Total CD': sweep_sweep_total_cd_output,
    #         'VLM drag (N)': sweep_sweep_vlm_drag_output,
    #         'Viscous drag (N)': sweep_sweep_viscous_drag_output,
    #         'Total drag (N)': sweep_sweep_total_drag_output,
    #     }
    # )
    # sweep_sweep_df.to_excel(f'TbwGeomDvSweeps_Sweep.xlsx')
    # print(sweep_sweep_df)
    # # endregion

    return


def trim_at_2_point_5g():
    caddee = cd.CADDEE()
    caddee.system_model = system_model = cd.SystemModel()
    caddee.system_representation = sys_rep = cd.SystemRepresentation()
    caddee.system_parameterization = sys_param = cd.SystemParameterization(system_representation=sys_rep)

    # region Geometry
    file_name = 'tbw.stp'

    spatial_rep = sys_rep.spatial_representation
    spatial_rep.import_file(file_name= GEOMETRY_FILES_FOLDER/ file_name)
    spatial_rep.refit_geometry(file_name=GEOMETRY_FILES_FOLDER / file_name)

    # region Lifting Surfaces
    # wing
    wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Wing']).keys())
    wing = LiftingSurface(name='wing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)
    if debug_geom_flag:
        wing.plot()
    sys_rep.add_component(wing)

    # Horizontal tail
    tail_primitive_names = list(spatial_rep.get_primitives(search_names=['Htail']).keys())
    htail = cd.LiftingSurface(name='h_tail', spatial_representation=spatial_rep, primitive_names=tail_primitive_names)
    if debug_geom_flag:
        htail.plot()
    sys_rep.add_component(htail)

    #Strut
    strut_primitive_names = list(spatial_rep.get_primitives(search_names=['Strut']).keys())
    strut = cd.LiftingSurface(name='strut', spatial_representation=spatial_rep, primitive_names=strut_primitive_names)
    if debug_geom_flag:
        strut.plot()
    sys_rep.add_component(strut)

    #jury
    jury_primitive_names = list(spatial_rep.get_primitives(search_names=['Jury']).keys())
    jury = cd.LiftingSurface(name='jury', spatial_representation=spatial_rep, primitive_names=jury_primitive_names)
    if debug_geom_flag:
        jury.plot()
    sys_rep.add_component(jury)
    # endregion
    # endregion

    # region Actuations
    # Tail FFD
    htail_geometry_primitives = htail.get_geometry_primitives()
    htail_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(
        htail_geometry_primitives, 
        num_control_points=(11, 2, 2), order=(4,2,2), 
        xyz_to_uvw_indices=(1,0,2))
    htail_ffd_block = cd.SRBGFFDBlock(name='htail_ffd_block', 
                                    primitive=htail_ffd_bspline_volume, 
                                    embedded_entities=htail_geometry_primitives)
    htail_ffd_block.add_scale_v(
        name='htail_linear_taper', 
        order=2, num_dof=3, value=np.array([0., 0., 0.]), 
        cost_factor=1.)
    htail_ffd_block.add_rotation_u(name='htail_twist_distribution', 
                                connection_name='h_tail_act', order=1, 
                                num_dof=1, value=np.array([np.deg2rad(1.75)]))
    # # NOTE: line above is performaing actuation- change when actuations are ready

    ffd_set = cd.SRBGFFDSet( 
        name='ffd_set', 
        ffd_blocks={htail_ffd_block.name : htail_ffd_block}
    )
    sys_param.add_geometry_parameterization(ffd_set)
    sys_param.setup()

    # endregion

    # region meshes

    num_spanwise = 10
    num_spanwise_strut = num_spanwise
    num_chordwise = 5
    num_chordwise_strut = num_chordwise

    # region wing mesh
    mesh_flag_wing = False
    num_wing_vlm = num_spanwise
    num_chordwise_vlm = num_chordwise
    point00 = np.array([68.035, 85.291, 4.704 + 0.1]) # * ft2m # Right tip leading edge
    point01 = np.array([71.790, 85.291, 4.708 + 0.1]) # * ft2m # Right tip trailing edge
    point10 = np.array([47.231,    0.000, 6.937 + 0.1]) # * ft2m # Center Leading Edge
    point11 = np.array([57.953,   0.000, 6.574 + 0.1]) # * ft2m # Center Trailing edge
    point20 = np.array([68.035, -85.291, 4.704 + 0.1]) # * ft2m # Left tip leading edge
    point21 = np.array([71.790, -85.291, 4.708 + 0.1]) # * ft2m # Left tip trailing edge

    do_plots = False

    leading_edge_points = np.concatenate((np.linspace(point00, point10, int(num_wing_vlm/2+1))[0:-1,:], np.linspace(point10, point20, int(num_wing_vlm/2+1))), axis=0)
    trailing_edge_points = np.concatenate((np.linspace(point01, point11, int(num_wing_vlm/2+1))[0:-1,:], np.linspace(point11, point21, int(num_wing_vlm/2+1))), axis=0)

    leading_edge = wing.project(leading_edge_points, direction=np.array([-1., 0., 0.]), plot=do_plots)
    trailing_edge = wing.project(trailing_edge_points, direction=np.array([1., 0., 0.]), plot=do_plots)

    # Chord Surface
    chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
    if mesh_flag_wing:
        spatial_rep.plot_meshes([chord_surface])

    # upper and lower surface
    wing_upper_surface_wireframe = wing.project(chord_surface.value + np.array([0., 0., 0.5]), direction=np.array([0., 0., -1.]), grid_search_n=25, plot=do_plots, max_iterations=200)
    wing_lower_surface_wireframe = wing.project(chord_surface.value - np.array([0., 0., 0.5]), direction=np.array([0., 0., 1.]), grid_search_n=25, plot=do_plots, max_iterations=200)

    # chamber surface
    wing_camber_surface = am.linspace(wing_upper_surface_wireframe, wing_lower_surface_wireframe, 1)
    wing_vlm_mesh_name = 'wing_vlm_mesh'
    sys_rep.add_output(wing_vlm_mesh_name, wing_camber_surface)
    if mesh_flag_wing:
        spatial_rep.plot_meshes([wing_camber_surface])

    # OML mesh
    wing_oml_mesh = am.vstack((wing_upper_surface_wireframe, wing_lower_surface_wireframe))
    wing_oml_mesh_name = 'wing_oml_mesh'
    sys_rep.add_output(wing_oml_mesh_name, wing_oml_mesh)
    if mesh_flag_wing:
        spatial_rep.plot_meshes([wing_oml_mesh])
    # endregion

    # region htail mesh
    plot_tail_mesh = False
    mesh_htail = False
    num_htail_vlm = num_spanwise
    num_chordwise_vlm = num_chordwise
    point00 = np.array([132.002-10.0, 19.217+4.5, 18.993+3.5]) # * ft2m # Right tip leading edge
    point01 = np.array([135.993, 19.217, 18.993]) # * ft2m # Right tip trailing edge
    point10 = np.array([122.905, 0.000, 20.000]) # * ft2m # Center Leading Edge
    point11 = np.array([134.308, 0.000, 20.000]) # * ft2m # Center Trailing edge
    point20 = np.array([132.002-10, -19.217-4.5, 18.993+3.5]) # * ft2m # Left tip leading edge
    point21 = np.array([135.993, -19.217, 18.993]) # * ft2m # Left tip trailing edge

    leading_edge_points = np.linspace(point00, point20, num_htail_vlm)
    trailing_edge_points = np.linspace(point01, point21, num_htail_vlm)

    leading_edge_htail = htail.project(leading_edge_points, direction=np.array([0., 0., -1.]), plot=plot_tail_mesh)
    trailing_edge_htail = htail.project(trailing_edge_points, direction=np.array([1., 0., 0.]), plot=plot_tail_mesh)


    # Chord Surface
    htail_chord_surface = am.linspace(leading_edge_htail, trailing_edge_htail, num_chordwise_vlm)
    if mesh_htail:
        spatial_rep.plot_meshes([htail_chord_surface])

    # Upper and Lower surface
    htail_upper_surface_wireframe = htail.project(htail_chord_surface.value + np.array([0., 0., 1.]), direction=np.array([0., 0., -1.]), grid_search_n=25, plot=plot_tail_mesh)
    htail_lower_surface_wireframe = htail.project(htail_chord_surface.value - np.array([0., 0., 1.]), direction=np.array([0., 0., 1.]), grid_search_n=25, plot=plot_tail_mesh)

    #chamber surface
    htail_camber_surface = am.linspace(htail_upper_surface_wireframe, htail_lower_surface_wireframe, 1)
    htail_vlm_mesh_name = 'htail_vlm_mesh'
    sys_rep.add_output(htail_vlm_mesh_name, htail_camber_surface)
    if mesh_htail:
        spatial_rep.plot_meshes([htail_camber_surface])

    # OML mesh
    htail_oml_mesh = am.vstack((htail_upper_surface_wireframe, htail_lower_surface_wireframe))
    htail_oml_mesh_name = 'htail_oml_mesh'
    sys_rep.add_output(htail_oml_mesh_name, htail_oml_mesh)
    if mesh_htail:
        spatial_rep.plot_meshes([htail_oml_mesh])
    # endregion

    # region strut mesh

    plot_strut_mesh = False
    num_spanwise_strut_vlm = num_spanwise_strut
    num_chordwise_strut_vlm = num_chordwise_strut
    vertex00 = np.array([55.573, -12.641, -4.200]) # left leading 1
    vertex06 = np.array([61.090, -48.994, 5.763]) # left leading 7
    vertex10 = np.array([57.309, -12.641, -4.200]) # left trailing 1
    vertex16 = np.array([62.902, -48.994, 5.763]) # left trailing 7
    vertex20 = np.array([55.573, 12.641, -4.200]) # right leading 1
    vertex26 = np.array([61.090, 48.994, 5.763]) # right leading 7
    vertex30 = np.array([57.309, 12.641, -4.200]) # right trailing 1
    vertex36 = np.array([62.902, 48.994, 5.763]) # right trailing 7

    do_plots_strut_leading = False
    do_plots_strut_trailing = False

    left_leading_edge = strut.project(np.linspace(vertex00, vertex06, num_spanwise_strut_vlm), direction=np.array([-1., 0., 0.]), plot=do_plots_strut_leading)
    right_leading_edge = strut.project(np.linspace(vertex20, vertex26, num_spanwise_strut_vlm), direction=np.array([-1., 0., 0.]), plot=do_plots_strut_leading)
    left_trailing_edge = strut.project(np.linspace(vertex10, vertex16, num_spanwise_strut_vlm) , direction=np.array([1., 0., 0.]), plot=do_plots_strut_trailing)
    right_trailing_edge = strut.project(np.linspace(vertex30, vertex36, num_spanwise_strut_vlm), direction=np.array([1., 0., 0.]), plot=do_plots_strut_trailing)

    do_strut_plot = False

    # region left strut mesh
    chord_surface_left = am.linspace(left_leading_edge, left_trailing_edge, num_chordwise_strut_vlm)
    if plot_strut_mesh:
        spatial_rep.plot_meshes([chord_surface_left])

    # Upper and Lower surface
    strut_left_upper_surface_wireframe = strut.project(chord_surface_left.value + np.array([0., 0., 0.5]), direction=np.array([0., 0., -1.]), grid_search_n=25, plot=do_strut_plot, max_iterations=200)
    strut_left_lower_surface_wireframe = strut.project(chord_surface_left.value - np.array([0., 0., 0.5]), direction=np.array([0., 0., 1.]), grid_search_n=25, plot=do_strut_plot, max_iterations=200)

    # Chamber surface
    strut_left_camber_surface = am.linspace(strut_left_upper_surface_wireframe, strut_left_lower_surface_wireframe, 1)
    strut_left_vlm_mesh_name = 'strut_vlm_mesh_left'
    sys_rep.add_output(strut_left_vlm_mesh_name, strut_left_camber_surface)
    if plot_strut_mesh:
        spatial_rep.plot_meshes([strut_left_camber_surface])

    # OML mesh
    oml_mesh = am.vstack((strut_left_upper_surface_wireframe, strut_left_lower_surface_wireframe))
    strut_left_oml_mesh_name = 'strut_oml_mesh'
    sys_rep.add_output(strut_left_oml_mesh_name, oml_mesh)
    if plot_strut_mesh:
        spatial_rep.plot_meshes([strut_left_camber_surface])
    # endregion

    # region right strut mesh
    chord_surface_right = am.linspace(right_leading_edge, right_trailing_edge, num_chordwise_strut_vlm)
    if plot_strut_mesh:
        spatial_rep.plot_meshes([chord_surface_right])

    # Upper and Lower surface
    strut_right_upper_surface_wireframe = strut.project(chord_surface_right.value + np.array([0., 0., 0.5]), direction=np.array([0., 0., -1.]), grid_search_n=25, plot=do_strut_plot, max_iterations=200)
    strut_right_lower_surface_wireframe = strut.project(chord_surface_right.value - np.array([0., 0., 0.5]), direction=np.array([0., 0., 1.]), grid_search_n=25, plot=do_strut_plot, max_iterations=200)

    # Chamber surface
    strut_right_camber_surface = am.linspace(strut_right_upper_surface_wireframe, strut_right_lower_surface_wireframe, 1)
    strut_right_vlm_mesh_name = 'strut_vlm_mesh_right'
    sys_rep.add_output(strut_right_vlm_mesh_name, strut_right_camber_surface)
    if plot_strut_mesh:
        spatial_rep.plot_meshes([strut_right_camber_surface])

    # OML Mesh
    strut_oml_mesh = am.vstack((strut_right_upper_surface_wireframe, strut_right_lower_surface_wireframe))
    strut_right_oml_mesh_name = 'strut_oml_mesh'
    sys_rep.add_output(strut_right_oml_mesh_name, strut_oml_mesh)
    if plot_strut_mesh:
        spatial_rep.plot_meshes([strut_right_camber_surface])

    plot_total_mesh = False

    if plot_total_mesh:
        spatial_rep.plot_meshes([strut_right_camber_surface, strut_left_camber_surface, wing_camber_surface, htail_camber_surface])

    # endregion

    # endregion

    # endregion

    ft2m = 0.3048
    # region Sizing
    tbw_wt = TBWMassProperties()
    mass, cg, I = tbw_wt.evaluate()

    total_mass_properties = cd.TotalMassPropertiesM3L()
    total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass, cg, I)

    # endregion

    # region design scenario
    design_scenario = cd.DesignScenario(name='aircraft_trim')

    # region cruise condtion
    cruise_model = m3l.Model()
    cruise_condition = cd.CruiseCondition(name="cruise_1")
    cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()
    cruise_condition.set_module_input(name='altitude', val=13106.4)
    cruise_condition.set_module_input(name='mach_number', val=0.70, dv_flag=False, lower=0.68, upper=0.72)
    cruise_condition.set_module_input(name='range', val=6482000)
    cruise_condition.set_module_input(name='pitch_angle', val=np.deg2rad(0), dv_flag=True, lower=0., upper=np.deg2rad(10))
    cruise_condition.set_module_input(name='flight_path_angle', val=0)
    cruise_condition.set_module_input(name='roll_angle', val=0)
    cruise_condition.set_module_input(name='yaw_angle', val=0)
    cruise_condition.set_module_input(name='wind_angle', val=0)
    cruise_condition.set_module_input(name='observer_location', val=np.array([0, 0, 500*ft2m]))

    ac_states = cruise_condition.evaluate_ac_states()
    cruise_model.register_output(ac_states)

    # region Aerodynamics

    vlm_model = VASTFluidSover(
        surface_names=[
            wing_vlm_mesh_name,
            htail_vlm_mesh_name,
            strut_left_vlm_mesh_name,
            strut_right_vlm_mesh_name
        ],
        surface_shapes=[
            (1, ) + wing_camber_surface.evaluate().shape[1:],
            (1, ) + htail_camber_surface.evaluate().shape[1:],
            (1, ) + strut_left_camber_surface.evaluate().shape[1:],
            (1, ) + strut_right_camber_surface.evaluate().shape[1:],
        ],
        fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
        mesh_unit='ft',
        cl0=[0.01,0.,0.,0.]
    )

    # aero forces and moments
    vlm_panel_forces, vlm_force, vlm_moment  = vlm_model.evaluate(ac_states=ac_states)
    cruise_model.register_output(vlm_force)
    cruise_model.register_output(vlm_moment)
    # endregion

    # region Propulsion loads
    ref_pt = np.array([0., 0., 2.8])
    tbw_left_prop_model = tbwPropulsionModel()
    tbw_left_prop_model.set_module_input('thrust_origin', val=np.array([0., 0., 6.256]))
    tbw_left_prop_model.set_module_input('ref_pt', val=ref_pt)
    tbw_left_prop_model.set_module_input('throttle', val = 1., dv_flag=True, lower=0., upper=1.)
    tbw_left_prop_forces, tbw_left_prop_moments = tbw_left_prop_model.evaluate(ac_states=ac_states)
    cruise_model.register_output(tbw_left_prop_forces)
    cruise_model.register_output(tbw_left_prop_moments)

    #region Engine Propulsion
    # region Left Engine Propulsion
    # ref_pt = np.array([0., 0., 2.8])
    # tbw_left_prop_model = cd.tbwPropulsionModel()
    # tbw_left_prop_model.set_module_input('thrust_origin', val=np.array([61.009, 42.646, 6.256]))
    # tbw_left_prop_model.set_module_input('ref_pt', val=ref_pt)
    # tbw_left_prop_model.set_module_input('throttle', val = 1., dv_flag=True, lower=0., upper=1.)
    # tbw_left_prop_forces, tbw_left_prop_moments = tbw_left_prop_model.evaluate(ac_states=ac_states)
    # cruise_model.register_output(tbw_left_prop_forces)
    # cruise_model.register_output(tbw_left_prop_moments)
    # endregion
    # region right Engine Propulsion
    # tbw_right_prop_model = cd.tbwPropulsionModel()
    # tbw_right_prop_model.set_module_input('thrust_origin', val=np.array([61.009, -42.646, 6.256]))
    # tbw_right_prop_model.set_module_input('ref_pt', val=ref_pt)
    # tbw_right_prop_model.set_module_input('throttle_right', val = 1., dv_flag=True, lower=0., upper=1.)
    # tbw_right_prop_forces, tbw_right_prop_moments = tbw_right_prop_model.evaluate(ac_states=ac_states)
    # cruise_model.register_output(tbw_right_prop_forces)
    # cruise_model.register_output(tbw_right_prop_moments)
    # endregion
    # endregion

    # endregion

    # region inertial loads
    inertial_loads_model = cd.InertialLoadsM3L(load_factor=2.5)
    inertial_forces, inertial_moments = inertial_loads_model.evaluate(total_cg_vector=total_cg, totoal_mass=total_mass, ac_states=ac_states)
    cruise_model.register_output(inertial_forces)
    cruise_model.register_output(inertial_moments)
    # endregion

    # Total Loads
    total_forces_moments_model = cd.TotalForcesMomentsM3L()
    total_forces, total_moments = total_forces_moments_model.evaluate(
        inertial_forces, inertial_moments,
        vlm_force, vlm_moment,
        tbw_left_prop_forces, tbw_left_prop_moments,
        #tbw_right_prop_forces, tbw_right_prop_moments
    )
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
    # endregion

    caddee_csdl_model = caddee.assemble_csdl()

    caddee_csdl_model.create_input(name='h_tail_act', val=np.deg2rad(0.))
    caddee_csdl_model.add_design_variable(dv_name='h_tail_act', lower=np.deg2rad(-10), upper=np.deg2rad(10), scaler=1.)


    # region Optimization Setup
    caddee_csdl_model.add_objective('system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual')
    # caddee_csdl_model.add_constraint(
    #     name='system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.total_CL',
    #     equals=0.766)
    # caddee_csdl_model.add_constraint(name='system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.L_over_D',
    #                                  equals=25.249,
    #                                  scaler = 1e-2
    #                                 )
    # endregion

    # create and run simulator
    sim = Simulator(caddee_csdl_model, analytics=True)
    # VLM - reference point 
    sim['system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.evaluation_pt'] = np.array([0., 0., 2.8])
    sim.run()
    #sim.compute_total_derivatives()
    #sim.check_totals()

    print('Total forces: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_forces'])
    print('Total moments:', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_moments'])
    print('Total_cl',sim['system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.total_CL'])

    prob = CSDLProblem(problem_name='lpc', simulator=sim)
    optimizer = SLSQP(prob, maxiter=1000, ftol=1E-10)
    optimizer.solve()
    optimizer.print_results()

    print('Trim residual: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.trim_residual'])
    print('Trim forces: ', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_forces'])
    print('Trim moments:', sim['system_model.aircraft_trim.cruise_1.cruise_1.euler_eom_gen_ref_pt.total_moments'])
    print('Horizontal tail actuation: ', np.rad2deg(sim['system_parameterization.ffd_set.rotational_section_properties_model.h_tail_act']))
    print('pitch: ', np.rad2deg(sim['system_model.aircraft_trim.cruise_1.cruise_1.cruise_1_ac_states_operation.cruise_1_pitch_angle']))
    print('Total_cl',sim['system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.total_CL'])
    print('throttle', sim['system_model.aircraft_trim.cruise_1.cruise_1.tbw_prop_model.throttle'])
    print('L_over_D',sim['system_model.aircraft_trim.cruise_1.cruise_1.wing_vlm_meshhtail_vlm_meshstrut_vlm_mesh_leftstrut_vlm_mesh_right_vlm_model.vast.VLMSolverModel.VLM_outputs.LiftDrag.L_over_D'])

def structural_wingbox_beam_evaluation():

    caddee = cd.CADDEE()
    caddee.system_model = system_model = cd.SystemModel()
    caddee.system_representation = sys_rep = cd.SystemRepresentation()
    caddee.system_parameterization = sys_param = cd.SystemParameterization(system_representation=sys_rep)

    # region Geometry
    file_name = 'tbw.stp'

    spatial_rep = sys_rep.spatial_representation
    spatial_rep.import_file(file_name= GEOMETRY_FILES_FOLDER/ file_name)
    spatial_rep.refit_geometry(file_name=GEOMETRY_FILES_FOLDER / file_name)

    # region Lifting Surfaces
    # wing
    wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Wing']).keys())
    wing = LiftingSurface(name='wing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)
    if debug_geom_flag:
        wing.plot()
    sys_rep.add_component(wing)

    # Horizontal tail
    tail_primitive_names = list(spatial_rep.get_primitives(search_names=['Htail']).keys())
    htail = cd.LiftingSurface(name='h_tail', spatial_representation=spatial_rep, primitive_names=tail_primitive_names)
    if debug_geom_flag:
        htail.plot()
    sys_rep.add_component(htail)

    #Strut
    strut_primitive_names = list(spatial_rep.get_primitives(search_names=['Strut']).keys())
    strut = cd.LiftingSurface(name='strut', spatial_representation=spatial_rep, primitive_names=strut_primitive_names)
    if debug_geom_flag:
        strut.plot()
    sys_rep.add_component(strut)

    #jury
    jury_primitive_names = list(spatial_rep.get_primitives(search_names=['Jury']).keys())
    jury = cd.LiftingSurface(name='jury', spatial_representation=spatial_rep, primitive_names=jury_primitive_names)
    if debug_geom_flag:
        jury.plot()
    sys_rep.add_component(jury)
    # endregion
    # endregion

    # region Actuations
    # Tail FFD
    htail_geometry_primitives = htail.get_geometry_primitives()
    htail_ffd_bspline_volume = cd.create_cartesian_enclosure_volume(
        htail_geometry_primitives, 
        num_control_points=(11, 2, 2), order=(4,2,2), 
        xyz_to_uvw_indices=(1,0,2))
    htail_ffd_block = cd.SRBGFFDBlock(name='htail_ffd_block', 
                                    primitive=htail_ffd_bspline_volume, 
                                    embedded_entities=htail_geometry_primitives)
    htail_ffd_block.add_scale_v(
        name='htail_linear_taper', 
        order=2, num_dof=3, value=np.array([0., 0., 0.]), 
        cost_factor=1.)
    htail_ffd_block.add_rotation_u(name='htail_twist_distribution', 
                                connection_name='h_tail_act', order=1, 
                                num_dof=1, value=np.array([np.deg2rad(1.75)]))
    # # NOTE: line above is performaing actuation- change when actuations are ready

    ffd_set = cd.SRBGFFDSet( 
        name='ffd_set', 
        ffd_blocks={htail_ffd_block.name : htail_ffd_block}
    )
    sys_param.add_geometry_parameterization(ffd_set)
    sys_param.setup()

    # endregion

    # region meshes

    num_spanwise = 10
    num_spanwise_strut = num_spanwise
    num_chordwise = 5
    num_chordwise_strut = num_chordwise

    # region wing mesh
    mesh_flag_wing = False
    num_wing_vlm = num_spanwise
    num_chordwise_vlm = num_chordwise
    point00 = np.array([68.035, 85.291, 4.704 + 0.1]) # * ft2m # Right tip leading edge
    point01 = np.array([71.790, 85.291, 4.708 + 0.1]) # * ft2m # Right tip trailing edge
    point10 = np.array([47.231,    0.000, 6.937 + 0.1]) # * ft2m # Center Leading Edge
    point11 = np.array([57.953,   0.000, 6.574 + 0.1]) # * ft2m # Center Trailing edge
    point20 = np.array([68.035, -85.291, 4.704 + 0.1]) # * ft2m # Left tip leading edge
    point21 = np.array([71.790, -85.291, 4.708 + 0.1]) # * ft2m # Left tip trailing edge

    do_plots = False

    leading_edge_points = np.concatenate((np.linspace(point00, point10, int(num_wing_vlm/2+1))[0:-1,:], np.linspace(point10, point20, int(num_wing_vlm/2+1))), axis=0)
    trailing_edge_points = np.concatenate((np.linspace(point01, point11, int(num_wing_vlm/2+1))[0:-1,:], np.linspace(point11, point21, int(num_wing_vlm/2+1))), axis=0)

    leading_edge = wing.project(leading_edge_points, direction=np.array([-1., 0., 0.]), plot=do_plots)
    trailing_edge = wing.project(trailing_edge_points, direction=np.array([1., 0., 0.]), plot=do_plots)

    # Chord Surface
    chord_surface = am.linspace(leading_edge, trailing_edge, num_chordwise_vlm)
    if mesh_flag_wing:
        spatial_rep.plot_meshes([chord_surface])

    # upper and lower surface
    wing_upper_surface_wireframe = wing.project(chord_surface.value + np.array([0., 0., 0.5]), direction=np.array([0., 0., -1.]), grid_search_n=25, plot=do_plots, max_iterations=200)
    wing_lower_surface_wireframe = wing.project(chord_surface.value - np.array([0., 0., 0.5]), direction=np.array([0., 0., 1.]), grid_search_n=25, plot=do_plots, max_iterations=200)

    # chamber surface
    wing_camber_surface = am.linspace(wing_upper_surface_wireframe, wing_lower_surface_wireframe, 1)
    wing_vlm_mesh_name = 'wing_vlm_mesh'
    sys_rep.add_output(wing_vlm_mesh_name, wing_camber_surface)
    if mesh_flag_wing:
        spatial_rep.plot_meshes([wing_camber_surface])

    # OML mesh
    wing_oml_mesh = am.vstack((wing_upper_surface_wireframe, wing_lower_surface_wireframe))
    wing_oml_mesh_name = 'wing_oml_mesh'
    sys_rep.add_output(wing_oml_mesh_name, wing_oml_mesh)
    if mesh_flag_wing:
        spatial_rep.plot_meshes([wing_oml_mesh])
    # endregion

    # region htail mesh
    plot_tail_mesh = False
    mesh_htail = False
    num_htail_vlm = num_spanwise
    num_chordwise_vlm = num_chordwise
    point00 = np.array([132.002-10.0, 19.217+4.5, 18.993+3.5]) # * ft2m # Right tip leading edge
    point01 = np.array([135.993, 19.217, 18.993]) # * ft2m # Right tip trailing edge
    point10 = np.array([122.905, 0.000, 20.000]) # * ft2m # Center Leading Edge
    point11 = np.array([134.308, 0.000, 20.000]) # * ft2m # Center Trailing edge
    point20 = np.array([132.002-10, -19.217-4.5, 18.993+3.5]) # * ft2m # Left tip leading edge
    point21 = np.array([135.993, -19.217, 18.993]) # * ft2m # Left tip trailing edge

    leading_edge_points = np.linspace(point00, point20, num_htail_vlm)
    trailing_edge_points = np.linspace(point01, point21, num_htail_vlm)

    leading_edge_htail = htail.project(leading_edge_points, direction=np.array([0., 0., -1.]), plot=plot_tail_mesh)
    trailing_edge_htail = htail.project(trailing_edge_points, direction=np.array([1., 0., 0.]), plot=plot_tail_mesh)


    # Chord Surface
    htail_chord_surface = am.linspace(leading_edge_htail, trailing_edge_htail, num_chordwise_vlm)
    if mesh_htail:
        spatial_rep.plot_meshes([htail_chord_surface])

    # Upper and Lower surface
    htail_upper_surface_wireframe = htail.project(htail_chord_surface.value + np.array([0., 0., 1.]), direction=np.array([0., 0., -1.]), grid_search_n=25, plot=plot_tail_mesh)
    htail_lower_surface_wireframe = htail.project(htail_chord_surface.value - np.array([0., 0., 1.]), direction=np.array([0., 0., 1.]), grid_search_n=25, plot=plot_tail_mesh)

    #chamber surface
    htail_camber_surface = am.linspace(htail_upper_surface_wireframe, htail_lower_surface_wireframe, 1)
    htail_vlm_mesh_name = 'htail_vlm_mesh'
    sys_rep.add_output(htail_vlm_mesh_name, htail_camber_surface)
    if mesh_htail:
        spatial_rep.plot_meshes([htail_camber_surface])

    # OML mesh
    htail_oml_mesh = am.vstack((htail_upper_surface_wireframe, htail_lower_surface_wireframe))
    htail_oml_mesh_name = 'htail_oml_mesh'
    sys_rep.add_output(htail_oml_mesh_name, htail_oml_mesh)
    if mesh_htail:
        spatial_rep.plot_meshes([htail_oml_mesh])
    # endregion

    # region strut mesh

    plot_strut_mesh = False
    num_spanwise_strut_vlm = num_spanwise_strut
    num_chordwise_strut_vlm = num_chordwise_strut
    vertex00 = np.array([55.573, -12.641, -4.200]) # left leading 1
    vertex06 = np.array([61.090, -48.994, 5.763]) # left leading 7
    vertex10 = np.array([57.309, -12.641, -4.200]) # left trailing 1
    vertex16 = np.array([62.902, -48.994, 5.763]) # left trailing 7
    vertex20 = np.array([55.573, 12.641, -4.200]) # right leading 1
    vertex26 = np.array([61.090, 48.994, 5.763]) # right leading 7
    vertex30 = np.array([57.309, 12.641, -4.200]) # right trailing 1
    vertex36 = np.array([62.902, 48.994, 5.763]) # right trailing 7

    do_plots_strut_leading = False
    do_plots_strut_trailing = False

    left_leading_edge = strut.project(np.linspace(vertex00, vertex06, num_spanwise_strut_vlm), direction=np.array([-1., 0., 0.]), plot=do_plots_strut_leading)
    right_leading_edge = strut.project(np.linspace(vertex20, vertex26, num_spanwise_strut_vlm), direction=np.array([-1., 0., 0.]), plot=do_plots_strut_leading)
    left_trailing_edge = strut.project(np.linspace(vertex10, vertex16, num_spanwise_strut_vlm) , direction=np.array([1., 0., 0.]), plot=do_plots_strut_trailing)
    right_trailing_edge = strut.project(np.linspace(vertex30, vertex36, num_spanwise_strut_vlm), direction=np.array([1., 0., 0.]), plot=do_plots_strut_trailing)

    do_strut_plot = False

    # region left strut mesh
    chord_surface_left = am.linspace(left_leading_edge, left_trailing_edge, num_chordwise_strut_vlm)
    if plot_strut_mesh:
        spatial_rep.plot_meshes([chord_surface_left])

    # Upper and Lower surface
    strut_left_upper_surface_wireframe = strut.project(chord_surface_left.value + np.array([0., 0., 0.5]), direction=np.array([0., 0., -1.]), grid_search_n=25, plot=do_strut_plot, max_iterations=200)
    strut_left_lower_surface_wireframe = strut.project(chord_surface_left.value - np.array([0., 0., 0.5]), direction=np.array([0., 0., 1.]), grid_search_n=25, plot=do_strut_plot, max_iterations=200)

    # Chamber surface
    strut_left_camber_surface = am.linspace(strut_left_upper_surface_wireframe, strut_left_lower_surface_wireframe, 1)
    strut_left_vlm_mesh_name = 'strut_vlm_mesh_left'
    sys_rep.add_output(strut_left_vlm_mesh_name, strut_left_camber_surface)
    if plot_strut_mesh:
        spatial_rep.plot_meshes([strut_left_camber_surface])

    # OML mesh
    oml_mesh = am.vstack((strut_left_upper_surface_wireframe, strut_left_lower_surface_wireframe))
    strut_left_oml_mesh_name = 'strut_oml_mesh'
    sys_rep.add_output(strut_left_oml_mesh_name, oml_mesh)
    if plot_strut_mesh:
        spatial_rep.plot_meshes([strut_left_camber_surface])
    # endregion

    # region right strut mesh
    chord_surface_right = am.linspace(right_leading_edge, right_trailing_edge, num_chordwise_strut_vlm)
    if plot_strut_mesh:
        spatial_rep.plot_meshes([chord_surface_right])

    # Upper and Lower surface
    strut_right_upper_surface_wireframe = strut.project(chord_surface_right.value + np.array([0., 0., 0.5]), direction=np.array([0., 0., -1.]), grid_search_n=25, plot=do_strut_plot, max_iterations=200)
    strut_right_lower_surface_wireframe = strut.project(chord_surface_right.value - np.array([0., 0., 0.5]), direction=np.array([0., 0., 1.]), grid_search_n=25, plot=do_strut_plot, max_iterations=200)

    # Chamber surface
    strut_right_camber_surface = am.linspace(strut_right_upper_surface_wireframe, strut_right_lower_surface_wireframe, 1)
    strut_right_vlm_mesh_name = 'strut_vlm_mesh_right'
    sys_rep.add_output(strut_right_vlm_mesh_name, strut_right_camber_surface)
    if plot_strut_mesh:
        spatial_rep.plot_meshes([strut_right_camber_surface])

    # OML Mesh
    strut_oml_mesh = am.vstack((strut_right_upper_surface_wireframe, strut_right_lower_surface_wireframe))
    strut_right_oml_mesh_name = 'strut_oml_mesh'
    sys_rep.add_output(strut_right_oml_mesh_name, strut_oml_mesh)
    if plot_strut_mesh:
        spatial_rep.plot_meshes([strut_right_camber_surface])

    plot_total_mesh = False

    if plot_total_mesh:
        spatial_rep.plot_meshes([strut_right_camber_surface, strut_left_camber_surface, wing_camber_surface, htail_camber_surface])

    # endregion

    # endregion

    # endregion

    ft2m = 0.3048
    # region Sizing
    tbw_wt = TBWMassProperties()
    mass, cg, I = tbw_wt.evaluate()

    total_mass_properties = cd.TotalMassPropertiesM3L()
    total_mass, total_cg, total_inertia = total_mass_properties.evaluate(mass, cg, I)

    # endregion
    debug_geom_flag = False
    visualize_flag = False
    num_wing_beam_nodes = 21
    r_tip_le = point00  # * ft2m # Right tip leading edge
    r_tip_te = point01 # * ft2m # Right tip trailing edge
    root_le = point10 # * ft2m # Center leading edge
    root_te = point11 # * ft2m # Center trailing edge
    l_tip_le = point20  # * ft2m # Left tip leading edge
    l_tip_te = point21 # * ft2m # Left tip trailing edge

    le_offset = np.array([-10, 0, 0])

    leading_edge_points = np.linspace(r_tip_le, l_tip_le, num_wing_beam_nodes)
    trailing_edge_points = np.vstack((np.linspace(r_tip_te, root_te, int(num_wing_beam_nodes / 2) + 1),
                                      np.linspace(root_te, l_tip_te, int(num_wing_beam_nodes / 2) + 1)[1:,
                                              :]))

    wing_leading_edge = wing.project(leading_edge_points + le_offset, direction=np.array([0., 0., -1]),
                                                plot=debug_geom_flag, force_reprojection = True)
    wing_trailing_edge = wing.project(trailing_edge_points, direction=np.array([1., 0., 0.]),
                                                    plot=debug_geom_flag, force_reprojection = True)


    wing_beam = am.linear_combination(wing_leading_edge, wing_trailing_edge, 1,
                                        start_weights=np.ones((num_wing_beam_nodes,)) * 0.75,
                                        stop_weights=np.ones((num_wing_beam_nodes,)) * 0.25)
    width = am.norm((wing_leading_edge - wing_trailing_edge) * 0.5)

    if debug_geom_flag:
                spatial_rep.plot_meshes([wing_beam])
    offset = np.array([0, 0, 0.5])
    top = wing.project(wing_beam.value + offset, direction=np.array([0., 0., -1.]), plot=debug_geom_flag)
    bot = wing.project(wing_beam.value - offset, direction=np.array([0., 0., 1.]), plot=debug_geom_flag)
    height = am.norm((top.reshape((-1, 3)) - bot.reshape((-1, 3))) * 1)


    sys_rep.add_output(name='wing_beam_mesh', quantity=wing_beam)
    sys_rep.add_output(name='wing_beam_width', quantity=width)
    sys_rep.add_output(name='wing_beam_height', quantity=height)

    # # pass the beam meshes to aframe:
    # beam_mesh = ebbeam.LinearBeamMesh(
    #     meshes=dict(
    #         wing_beam=wing_beam,
    #         wing_beam_width=width,
    #         wing_beam_height=height, ))
    #         self.mesh_data['beam']['ebbeam'] = beam_mesh

    # # pass the beam meshes to the aframe mass model:
    # beam_mass_mesh = MassMesh(
    #     meshes=dict(
    #         wing_beam=wing_beam,
    #         wing_beam_width=width,
    #         wing_beam_height=height, ))
    # self.mesh_data['beam']['mass'] = beam_mass_mesh

    # if visualize_flag:
    #             spatial_rep.plot_meshes([wing_beam])    
    
    exit()
    # region design scenario
    design_scenario = cd.DesignScenario(name='aircraft_trim')

    # region cruise condtion
    cruise_model = m3l.Model()
    cruise_condition = cd.CruiseCondition(name="cruise_1")
    cruise_condition.atmosphere_model = cd.SimpleAtmosphereModel()
    cruise_condition.set_module_input(name='altitude', val=13106.4)
    cruise_condition.set_module_input(name='mach_number', val=0.70, dv_flag=False, lower=0.68, upper=0.72)
    cruise_condition.set_module_input(name='range', val=6482000)
    cruise_condition.set_module_input(name='pitch_angle', val=np.deg2rad(0), dv_flag=True, lower=0., upper=np.deg2rad(10))
    cruise_condition.set_module_input(name='flight_path_angle', val=0)
    cruise_condition.set_module_input(name='roll_angle', val=0)
    cruise_condition.set_module_input(name='yaw_angle', val=0)
    cruise_condition.set_module_input(name='wind_angle', val=0)
    cruise_condition.set_module_input(name='observer_location', val=np.array([0, 0, 500*ft2m]))

    ac_states = cruise_condition.evaluate_ac_states()
    cruise_model.register_output(ac_states)

    # region Aerodynamics

    vlm_model = VASTFluidSover(
        surface_names=[
            wing_vlm_mesh_name,
            htail_vlm_mesh_name,
            strut_left_vlm_mesh_name,
            strut_right_vlm_mesh_name
        ],
        surface_shapes=[
            (1, ) + wing_camber_surface.evaluate().shape[1:],
            (1, ) + htail_camber_surface.evaluate().shape[1:],
            (1, ) + strut_left_camber_surface.evaluate().shape[1:],
            (1, ) + strut_right_camber_surface.evaluate().shape[1:],
        ],
        fluid_problem=FluidProblem(solver_option='VLM', problem_type='fixed_wake'),
        mesh_unit='ft',
        cl0=[0.01, 0., 0., 0.]
    )

    # aero forces and moments
    vlm_panel_forces, vlm_force, vlm_moment  = vlm_model.evaluate(ac_states=ac_states)
    cruise_model.register_output(vlm_force)
    cruise_model.register_output(vlm_moment)
    
    vlm_force_mapping_model = VASTNodalForces(
        surface_names=[
            wing_vlm_mesh_name,
            htail_vlm_mesh_name,
            strut_left_vlm_mesh_name,
            strut_right_vlm_mesh_name
        ],
        surface_shapes=[
            (1, ) + wing_camber_surface.evaluate().shape[1:],
            (1, ) + htail_camber_surface.evaluate().shape[1:],
            (1, ) + strut_left_camber_surface.evaluate().shape[1:],
            (1, ) + strut_right_camber_surface.evaluate().shape[1:],
            ],
        initial_meshes=[
            wing_camber_surface,
            htail_camber_surface,
            strut_left_camber_surface,
            strut_right_camber_surface]
    )

    oml_forces = vlm_force_mapping_model.evaluate(vlm_forces=vlm_panel_forces, nodal_force_meshes=[oml_mesh, oml_mesh])
    wing_forces = oml_forces[0]
    htail_forces = oml_forces[1]
    strut_left_forces = oml_forces[2]
    strut_right_forces = oml_forces[3]
    
    # endregion
    # endregion
    # endregion




if __name__ == '__main__':
    # trim_at_1g()
    shape_vars_sweeps_at_1g()
    # trim_at_2_point_5g()
    #structural_wingbox_beam_evaluation()
