# region Imports
import vedo
import caddee.api as cd
import m3l
from python_csdl_backend import Simulator
from modopt.snopt_library import SNOPT
from modopt.scipy_library import SLSQP
from modopt.optimization_algorithms import SQP
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
plot_total_mesh = True

tip_translate = 0.
wing_span = 140.


if __name__ == '__main__':
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
                                                value=np.array([tip_translate, 0., tip_translate]),
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
    sys_param.add_input(name='wing_span', quantity=span_tip_leading_edge, value=wing_span)
    # endregion

    sys_param.setup()

    # region meshes

    num_spanwise_wing = 21
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

    # if plot_total_mesh:
    #     spatial_rep.plot_meshes(
    #         [strut_right_camber_surface, strut_left_camber_surface, wing_camber_surface, htail_camber_surface])

    # endregion

    # endregion

    # endregion

    system_representation_model = sys_rep.assemble_csdl()
    system_parameterization_model = sys_param.assemble_csdl()

    my_model = csdl.Model()
    my_model.add(system_parameterization_model, 'system_parameterization')
    my_model.add(system_representation_model, 'system_representation')

    sim = Simulator(my_model, analytics=True, display_scripts=True)
    sim.run()

    if plot_total_mesh:
        geom = sim['design_geometry']
        spatial_rep.update(geom)
        strut_right_camber_surface.evaluate(input=geom)
        strut_left_camber_surface.evaluate(input=geom)
        wing_camber_surface.evaluate(input=geom)
        htail_camber_surface.evaluate(input=geom)
        plot_data = spatial_rep.plot_meshes(
            [strut_right_camber_surface, strut_left_camber_surface, wing_camber_surface, htail_camber_surface],
        show=False)
        plotter = vedo.Plotter(size=(3200, 2000), offscreen=True)
        center_x = 60  # eyeballing center x coordinate of geometry
        center_z = 0  # eyeballing center z coordinate of geometry
        camera_settings = {
            'pos': (-62, 98, 70),
            'viewup': (0, 0, 1),
            'focalPoint': (center_x - 8, 0 + 20, center_z + 5)
        }
        plotter.show(plot_data, 'Meshes', axes=None,
                     interactive=False,
                     camera=camera_settings)
        plotter.screenshot(filename='temp.png')

