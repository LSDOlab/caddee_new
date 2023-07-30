from caddee import GEOMETRY_FILES_FOLDER
import caddee.api as cd

import numpy as np
from typing import Type

from caddee.core.caddee_core.system_representation.component.component import LiftingSurface, Component
from aframe.core.mass import MassMesh
import aframe.core.beam_module as ebbeam
import array_mapper as am


in2m = 0.0254
ft2m = 0.3048
lbs2kg = 0.453592
psf2pa = 50


class PavGeomMesh:
    def __init__(self):
        self.stp_file_name = 'pav.stp'
        self.sys_rep = cd.SystemRepresentation()
        self.sys_param = cd.SystemParameterization(system_representation=self.sys_rep)
        self.geom_data = {
            'components': {
                'wing': Type[cd.LiftingSurface],
                'htail': Type[cd.LiftingSurface]
            },
            'primitive_names': {
                'wing': list(), 'left_wing': list(), 'right_wing': list(),
                'htail': list(),
            }
        }
        self.mesh_data ={
            'oml': {
                'oml_surface': {
                    'wing': None, 'htail': None
                },
                'mesh_name': {
                    'wing': Type[str], 'htail': Type[str]
                }
            },
            'vlm': {
                'chamber_surface': {
                    'wing': None, 'htail': None
                },
                'mesh_name': {
                    'wing': Type[str], 'htail': Type[str]
                }
            },
            'beam': {
                'ebbeam': Type[ebbeam.LinearBeamMesh],
                'mass': Type[MassMesh]
            }
        }
        return

    def setup_geometry(
            self,
            include_wing_flag=False,
            include_htail_flag=False,
            debug_geom_flag = False,
    ):

        spatial_rep = self.sys_rep.spatial_representation
        spatial_rep.import_file(file_name=GEOMETRY_FILES_FOLDER / self.stp_file_name)
        spatial_rep.refit_geometry(file_name=GEOMETRY_FILES_FOLDER / self.stp_file_name)

        # Fix naming
        primitives_new = {}
        indicies_new = {}
        for key, item in spatial_rep.primitives.items():
            item.name = item.name.replace(' ', '_').replace(',', '')
            primitives_new[key.replace(' ', '_').replace(',', '')] = item

        for key, item in spatial_rep.primitive_indices.items():
            indicies_new[key.replace(' ', '_').replace(',', '')] = item

        spatial_rep.primitives = primitives_new
        spatial_rep.primitive_indices = indicies_new

        # region Lifting Surfaces
        # region Wing

        # Left Wing
        left_wing_names = []
        left_wing_top_names = []
        left_wing_bottom_names = []
        left_wing_te_top_names = []
        left_wing_te_bottom_names = []
        for i in range(22 + 172, 37 + 172):
            surf_name = 'Wing_1_' + str(i)
            left_wing_names.append(surf_name)
            if i % 4 == 2:
                left_wing_te_bottom_names.append(surf_name)
            elif i % 4 == 3:
                left_wing_bottom_names.append(surf_name)
            elif i % 4 == 0:
                left_wing_top_names.append(surf_name)
            else:
                left_wing_te_top_names.append(surf_name)

        # Right Wing
        right_wing_names = []
        right_wing_top_names = []
        right_wing_bottom_names = []
        right_wing_te_top_names = []
        right_wing_te_bottom_names = []
        for i in range(174, 189):
            surf_name = 'Wing_0_' + str(i)
            right_wing_names.append(surf_name)
            if i % 4 == 2:
                right_wing_te_bottom_names.append(surf_name)
            elif i % 4 == 3:
                right_wing_bottom_names.append(surf_name)
            elif i % 4 == 0:
                right_wing_top_names.append(surf_name)
            else:
                right_wing_te_top_names.append(surf_name)

        # Components
        wing_left = LiftingSurface(name='wing_left', spatial_representation=spatial_rep, primitive_names=left_wing_names)
        wing_left_top = LiftingSurface(name='wing_left_top', spatial_representation=spatial_rep,
                                       primitive_names=left_wing_top_names)
        wing_left_bottom = LiftingSurface(name='wing_left_bottom', spatial_representation=spatial_rep,
                                          primitive_names=left_wing_bottom_names)
        wing_left_te = LiftingSurface(name='wing_te', spatial_representation=spatial_rep,
                                      primitive_names=left_wing_te_top_names + left_wing_te_bottom_names)

        wing_oml = LiftingSurface(name='wing_oml', spatial_representation=spatial_rep,
                                  primitive_names=left_wing_names + right_wing_names)
        wing_top = LiftingSurface(name='wing_top', spatial_representation=spatial_rep,
                                  primitive_names=left_wing_top_names + right_wing_top_names)
        wing_bottom = LiftingSurface(name='wing_left_bottom', spatial_representation=spatial_rep,
                                     primitive_names=left_wing_bottom_names + right_wing_bottom_names)
        wing_te = LiftingSurface(name='wing_te', spatial_representation=spatial_rep,
                                 primitive_names=left_wing_te_top_names + left_wing_te_bottom_names + right_wing_te_top_names + right_wing_te_bottom_names)

        if include_wing_flag:
            wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Wing']).keys())
            wing = LiftingSurface(name='Wing', spatial_representation=spatial_rep, primitive_names=wing_primitive_names)
            if debug_geom_flag:
                wing.plot()
            self.sys_rep.add_component(wing)
            self.geom_data['components']['wing'] = wing
            self.geom_data['primitive_names']['wing'] = wing_primitive_names

            self.geom_data['primitive_names']['left_wing'] = left_wing_names
            self.geom_data['primitive_names']['right_wing'] = right_wing_names

            self.geom_data['components']['wing_te'] = wing_te

        # endregion

        # region Horizontal Tail
        if include_htail_flag:
            tail_primitive_names = list(spatial_rep.get_primitives(search_names=['Stabilizer']).keys())
            htail = cd.LiftingSurface(name='HTail', spatial_representation=spatial_rep, primitive_names=tail_primitive_names)
            if debug_geom_flag:
                htail.plot()
            self.sys_rep.add_component(htail)
            self.geom_data['components']['htail'] = htail
            self.geom_data['primitive_names']['htail'] = tail_primitive_names
        # endregion
        # endregion
        return

    def setup_index_functions(self):

        # wing force function
        num = 25
        u, v = np.meshgrid(np.linspace(0, 1, num), np.linspace(0, 1, num))
        u = np.array(u).flatten()
        v = np.array(v).flatten()
        points = np.vstack((u, v)).T
        space_f = IDWFunctionSpace(name='force_base_space', points=points, order=1,
                                   coefficients_shape=(points.shape[0],))
        wing_force = index_functions(
            self.geom_data['primitive_names']['left_wing'] + self.geom_data['primitive_names']['right_wing'],
            'wing_force',
            space_f,
            3)
        return

    def vlm_meshes(
            self,
            include_wing_flag=False, num_wing_spanwise_vlm=21, num_wing_chordwise_vlm=5,
            include_htail_flag=False, num_htail_spanwise_vlm=21, num_htail_chordwise_vlm=5,
            debug_geom_flag=False, visualize_flag=False,
            force_reprojection=False
    ):
        spatial_rep = self.sys_rep.spatial_representation

        # region Wing
        if include_wing_flag:

            wing_component = self.geom_data['components']['wing']
            wing_te_component = self.geom_data['components']['wing_te']

            root_te = np.array([15.170, 0., 1.961]) * ft2m
            root_le = np.array([8.800, 0, 1.989]) * ft2m
            l_tip_te = np.array([11.300, -14.000, 1.978]) * ft2m
            l_tip_le = np.array([8.796, -14.000, 1.989]) * ft2m
            r_tip_te = np.array([11.300, 14.000, 1.978]) * ft2m
            r_tip_le = np.array([8.796, 14.000, 1.989]) * ft2m


            le_offset = np.array([-10, 0, 0])
            te_offset = np.array([10, 0, 0])

            leading_edge_points = np.linspace(r_tip_le, l_tip_le, num_wing_spanwise_vlm)
            trailing_edge_points = np.vstack((np.linspace(r_tip_te, root_te, int(num_wing_spanwise_vlm / 2) + 1),
                                              np.linspace(root_te, l_tip_te, int(num_wing_spanwise_vlm / 2) + 1)[1:, :]))

            leading_edge = wing_component.project(leading_edge_points + le_offset, direction=np.array([0., 0., -1]),
                                                  plot=debug_geom_flag, force_reprojection=force_reprojection)
            trailing_edge = wing_te_component.project(trailing_edge_points, direction=np.array([1., 0., 0.]),
                                                      plot=debug_geom_flag, force_reprojection=force_reprojection)

            # Chord Surface
            wing_chord_surface = am.linspace(leading_edge, trailing_edge, num_wing_chordwise_vlm)
            if debug_geom_flag:
                spatial_rep.plot_meshes([wing_chord_surface])

            # Upper and lower surface
            wing_upper_surface_wireframe = wing_component.project(wing_chord_surface.value + np.array([0., 0., 0.5 * ft2m]),
                                                            direction=np.array([0., 0., -1.]), grid_search_n=25,
                                                            plot=debug_geom_flag, max_iterations=200,
                                                            force_reprojection=force_reprojection)
            wing_lower_surface_wireframe = wing_component.project(wing_chord_surface.value - np.array([0., 0., 0.5 * ft2m]),
                                                            direction=np.array([0., 0., 1.]), grid_search_n=25,
                                                            plot=debug_geom_flag, max_iterations=200,
                                                            force_reprojection=force_reprojection)

            # Chamber surface
            wing_camber_surface = am.linspace(wing_upper_surface_wireframe, wing_lower_surface_wireframe, 1)
            wing_vlm_mesh_name = 'wing_vlm_mesh'
            self.sys_rep.add_output(wing_vlm_mesh_name, wing_camber_surface)
            if debug_geom_flag:
                spatial_rep.plot_meshes([wing_camber_surface])

            # Chord distribution
            self.sys_rep.add_output(name='wing_chord_distribution',
                               quantity=am.norm(leading_edge - trailing_edge))

            # # OML mesh
            # surfaces = self.geom_data['primitive_names']['left_wing'] + self.geom_data['primitive_names']['right_wing']
            # oml_para_mesh = []
            # grid_num = 10
            # for name in surfaces:
            #     for u in np.linspace(0, 1, grid_num):
            #         for v in np.linspace(0, 1, grid_num):
            #             oml_para_mesh.append((name, np.array([u, v]).reshape((1, 2))))
            #
            # oml_geo_mesh = spatial_rep.evaluate_parametric(oml_para_mesh)
            # wing_oml_mesh_name = 'wing_oml_mesh'
            # self.sys_rep.add_output(wing_oml_mesh_name, oml_geo_mesh)
            # if debug_geom_flag:
            #     spatial_rep.plot_meshes([oml_geo_mesh])

            # OML mesh
            wing_oml_mesh = am.vstack((wing_upper_surface_wireframe, wing_lower_surface_wireframe))
            wing_oml_mesh_name = 'wing_oml_mesh'
            self.sys_rep.add_output(wing_oml_mesh_name, wing_oml_mesh)
            self.mesh_data['oml']['oml_surface']['wing'] = wing_oml_mesh
            self.mesh_data['oml']['mesh_name']['wing'] = wing_oml_mesh_name

            if debug_geom_flag:
                spatial_rep.plot_meshes([wing_oml_mesh])

            self.mesh_data['vlm']['chamber_surface']['wing'] = wing_camber_surface
            self.mesh_data['vlm']['mesh_name']['wing'] = wing_vlm_mesh_name

        # endregion

        # region Tail
        if include_htail_flag:
            htail_component = self.geom_data['components']['htail']

            point00 = np.array([20.713 - 4., 8.474 + 1.5, 0.825 + 1.5]) * ft2m # Right tip leading edge
            point01 = np.array([22.916, 8.474, 0.825])  * ft2m # Right tip trailing edge
            point10 = np.array([18.085, 0.000, 0.825])  * ft2m # Center Leading Edge
            point11 = np.array([23.232, 0.000, 0.825])  * ft2m # Center Trailing edge
            point20 = np.array([20.713 - 4., -8.474 - 1.5, 0.825 + 1.5])  * ft2m # Left tip leading edge
            point21 = np.array([22.916, -8.474, 0.825])  * ft2m # Left tip trailing edge

            leading_edge_points = np.linspace(point00, point20, num_htail_spanwise_vlm)
            trailing_edge_points = np.linspace(point01, point21, num_htail_spanwise_vlm)

            leading_edge = htail_component.project(leading_edge_points, direction=np.array([0., 0., -1.]),
                                                   force_reprojection=force_reprojection,
                                                   plot=debug_geom_flag)
            trailing_edge = htail_component.project(trailing_edge_points, direction=np.array([1., 0., 0.]),
                                                    force_reprojection=force_reprojection,
                                                    plot=debug_geom_flag)

            # Chord Surface
            htail_chord_surface = am.linspace(leading_edge, trailing_edge, num_htail_chordwise_vlm)
            if debug_geom_flag:
                spatial_rep.plot_meshes([htail_chord_surface])

            # Upper and lower surface
            htail_upper_surface_wireframe = htail_component.project(
                htail_chord_surface.value + np.array([0., 0., 0.5 * ft2m]),
                direction=np.array([0., 0., -1.]), grid_search_n=25,
                plot=debug_geom_flag, max_iterations=200,
                force_reprojection=force_reprojection)
            htail_lower_surface_wireframe = htail_component.project(
                htail_chord_surface.value - np.array([0., 0., 0.5 * ft2m]),
                direction=np.array([0., 0., 1.]), grid_search_n=25,
                plot=debug_geom_flag, max_iterations=200,
                force_reprojection=force_reprojection)

            # Chamber surface
            htail_camber_surface = am.linspace(htail_upper_surface_wireframe, htail_lower_surface_wireframe, 1)
            htail_vlm_mesh_name = 'htail_vlm_mesh'
            self.sys_rep.add_output(htail_vlm_mesh_name, htail_camber_surface)
            if debug_geom_flag:
                spatial_rep.plot_meshes([htail_camber_surface])

            # OML mesh
            htail_oml_mesh = am.vstack((htail_upper_surface_wireframe, htail_lower_surface_wireframe))
            htail_oml_mesh_name = 'htail_oml_mesh'
            self.sys_rep.add_output(htail_oml_mesh_name, htail_oml_mesh)
            if debug_geom_flag:
                spatial_rep.plot_meshes([htail_oml_mesh])

            self.mesh_data['vlm']['chamber_surface']['htail'] = htail_camber_surface
            self.mesh_data['vlm']['mesh_name']['htail'] = htail_vlm_mesh_name
        # endregion

        if visualize_flag:
            if include_wing_flag and not include_htail_flag :  # Wing-Only
                spatial_rep.plot_meshes([wing_camber_surface])
            elif include_htail_flag and not include_wing_flag:
                raise NotImplementedError
            elif include_wing_flag and include_htail_flag :
                spatial_rep.plot_meshes([wing_camber_surface, htail_camber_surface])
        return

    def actuations(self,
                   include_tail_actuation_flag):
        if include_tail_actuation_flag:
            # Tail FFD
            htail_geometry_primitives = self.geom_data['components']['htail'].get_geometry_primitives()
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
            self.sys_param.add_geometry_parameterization(ffd_set)
        # endregion
        return

    def beam_mesh(self,
                  include_wing_flag=False, num_wing_beam_nodes=21,
                  debug_geom_flag=False, visualize_flag=False,
                  force_reprojection=True
                  ):

        spatial_rep = self.sys_rep.spatial_representation

        if include_wing_flag:

            wing_component = self.geom_data['components']['wing']
            wing_te_component = self.geom_data['components']['wing_te']

            root_te = np.array([15.170, 0., 1.961]) * ft2m
            root_le = np.array([8.800, 0, 1.989]) * ft2m
            l_tip_te = np.array([11.300, -14.000, 1.978]) * ft2m
            l_tip_le = np.array([8.796, -14.000, 1.989]) * ft2m
            r_tip_te = np.array([11.300, 14.000, 1.978]) * ft2m
            r_tip_le = np.array([8.796, 14.000, 1.989]) * ft2m

            le_offset = np.array([-10, 0, 0])

            leading_edge_points = np.linspace(r_tip_le, l_tip_le, num_wing_beam_nodes)
            trailing_edge_points = np.vstack((np.linspace(r_tip_te, root_te, int(num_wing_beam_nodes / 2) + 1),
                                              np.linspace(root_te, l_tip_te, int(num_wing_beam_nodes / 2) + 1)[1:,
                                              :]))

            wing_leading_edge = wing_component.project(leading_edge_points + le_offset, direction=np.array([0., 0., -1]),
                                                  plot=debug_geom_flag, force_reprojection=force_reprojection)
            wing_trailing_edge = wing_te_component.project(trailing_edge_points, direction=np.array([1., 0., 0.]),
                                                      plot=debug_geom_flag, force_reprojection=force_reprojection)


            wing_beam = am.linear_combination(wing_leading_edge, wing_trailing_edge, 1,
                                              start_weights=np.ones((num_wing_beam_nodes,)) * 0.75,
                                              stop_weights=np.ones((num_wing_beam_nodes,)) * 0.25)
            width = am.norm((wing_leading_edge - wing_trailing_edge) * 0.5)

            if debug_geom_flag:
                spatial_rep.plot_meshes([wing_beam])

            offset = np.array([0, 0, 0.5])
            top = wing_component.project(wing_beam.value + offset, direction=np.array([0., 0., -1.]), plot=debug_geom_flag)
            bot = wing_component.project(wing_beam.value - offset, direction=np.array([0., 0., 1.]), plot=debug_geom_flag)
            height = am.norm((top.reshape((-1, 3)) - bot.reshape((-1, 3))) * 1)

            self.sys_rep.add_output(name='wing_beam_mesh', quantity=wing_beam)
            self.sys_rep.add_output(name='wing_beam_width', quantity=width)
            self.sys_rep.add_output(name='wing_beam_height', quantity=height)

            # pass the beam meshes to aframe:
            beam_mesh = ebbeam.LinearBeamMesh(
                meshes=dict(
                    wing_beam=wing_beam,
                    wing_beam_width=width,
                    wing_beam_height=height, ))
            self.mesh_data['beam']['ebbeam'] = beam_mesh

            # pass the beam meshes to the aframe mass model:
            beam_mass_mesh = MassMesh(
                meshes=dict(
                    wing_beam=wing_beam,
                    wing_beam_width=width,
                    wing_beam_height=height, ))
            self.mesh_data['beam']['mass'] = beam_mass_mesh

            if visualize_flag:
                spatial_rep.plot_meshes([wing_beam])
        return