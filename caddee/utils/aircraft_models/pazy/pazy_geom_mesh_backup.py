from caddee import GEOMETRY_FILES_FOLDER
import caddee.api as cd

import numpy as np
from typing import Type

from caddee.core.caddee_core.system_representation.component.component import LiftingSurface, Component
from aframe.core.mass import MassMesh
import aframe.core.beam_module as ebbeam
import array_mapper as am
from caddee.core.caddee_core.system_representation.system_primitive.system_primitive import SystemPrimitive
import caddee.core.primitives.bsplines.bspline_functions as bsf
import lsdo_geo as lg

import m3l
from m3l.core.function_spaces import IDWFunctionSpace
from m3l.utils.utils import index_functions


in2m = 0.0254
ft2m = 0.3048
lbs2kg = 0.453592
psf2pa = 50


class PazyGeomMesh:
    def __init__(self):
        self.stp_file_name = 'PazyWing.stp'
        self.sys_rep = cd.SystemRepresentation()
        self.sys_param = cd.SystemParameterization(system_representation=self.sys_rep)
        self.geom_data = {
            'components': {
                'wing': Type[cd.LiftingSurface],
                # 'htail': Type[cd.LiftingSurface]
            },
            'primitive_names': {
                'wing': list(), 'left_wing': list(), 'right_wing': list(),
                # 'htail': list(),
            },
            'points': {
                'wing': {
                    'root_te': Type[np.array], 'l_tip_te': Type[np.array], 'r_tip_te': Type[np.array],
                    'root_le': Type[np.array], 'l_tip_le': Type[np.array], 'r_tip_le': Type[np.array]
                }#,
                # 'htail': {
                #     'root_te': Type[np.array], 'l_tip_te': Type[np.array], 'r_tip_te': Type[np.array],
                #     'root_le': Type[np.array], 'l_tip_le': Type[np.array], 'r_tip_le': Type[np.array]
                # }
            }
        }
        self.mesh_data ={
            'oml': {
                'oml_geo_nodes': {
                    'wing': None#, 'htail': None
                },
                'oml_para_nodes': {
                    'wing': Type[list]#, 'htail': Type[list]
                },
                'mesh_name': {
                    'wing': Type[str]#, 'htail': Type[str]
                }
            },
            'vlm': {
                'chamber_surface': {
                    'wing': None#, 'htail': None
                },
                'mesh_name': {
                    'wing': Type[str]#, 'htail': Type[str]
                }
            },
            'beam': {
                'ebbeam': Type[ebbeam.LinearBeamMesh],
                'mass': Type[MassMesh]
            },
            'ml': {
                'wing_upper': None, 'wing_lower': None,
                'wing_upper_vlm': None, 'wing_lower_vlm':None,
                'wing_upper_parametric':None, 'wing_lower_parametric':None
            }
        }
        self.functions = {
            'wing_thickness': None,
            'wing_displacement_input': None,
            'wing_displacement_output': None,
            'wing_force': None,
            'wing_cp': None
        }
        return

    def setup_geometry(
            self,
            include_wing_flag=False,
            include_htail_flag=False,
            debug_geom_flag = False,
            force_reprojection=False
    ):

        spatial_rep = self.sys_rep.spatial_representation
        spatial_rep.import_file(file_name=GEOMETRY_FILES_FOLDER / self.stp_file_name)
        spatial_rep.refit_geometry(file_name=GEOMETRY_FILES_FOLDER / self.stp_file_name)

        # Fix naming
        primitives_new = {}
        indicies_new = {}
        for key, item in spatial_rep.primitives.items():
            # we filter the surfaces with index 2 or 3 since these are the only wing surfaces
            if '2' in key or '3' in key:
                item.name = item.name.replace(' ', '_').replace(',', '')
                primitives_new[key.replace(' ', '_').replace(',', '')] = item

        for key, item in spatial_rep.primitive_indices.items():
            if '2' in key or '3' in key:
                indicies_new[key.replace(' ', '_').replace(',', '')] = item

        spatial_rep.primitives = primitives_new
        spatial_rep.primitive_indices = indicies_new

        # region Lifting Surfaces
        # region Wing
        if include_wing_flag:
            # Right Wing
            right_wing_names = []
            right_wing_top_names = []
            right_wing_bottom_names = []
            ring_wing_outboard_closure_names = []
            right_wing_inboard_closure_names = []
            for i in range(0, 6):
                surf_name = 'PazyWingGeom_0_' + str(i)
                # We explicitly ignore the inboard and outboard closure surfaces
                if i in [2, 3]:
                    right_wing_names.append(surf_name)

                if i in [0, 1]:
                    ring_wing_outboard_closure_names.append(surf_name)
                elif i == 2:
                    right_wing_bottom_names.append(surf_name)
                elif i == 3:
                    right_wing_top_names.append(surf_name)
                else:
                    right_wing_inboard_closure_names.append(surf_name)

            # Components
            wing_oml = LiftingSurface(name='wing_oml', spatial_representation=spatial_rep,
                                      primitive_names=right_wing_names)
            wing_top = LiftingSurface(name='wing_top', spatial_representation=spatial_rep,
                                      primitive_names=right_wing_top_names)
            wing_bottom = LiftingSurface(name='wing_bottom', spatial_representation=spatial_rep,
                                         primitive_names=right_wing_bottom_names)
            # wing_te = LiftingSurface(name='wing_te', spatial_representation=spatial_rep,
            #                          primitive_names=left_wing_te_top_names + left_wing_te_bottom_names + right_wing_te_top_names + right_wing_te_bottom_names)

            self.geom_data['points']['wing']['root_te'] = np.array([0.1, 0., 0.])
            self.geom_data['points']['wing']['root_le'] = np.array([0., 0., 0.])
            self.geom_data['points']['wing']['r_tip_te'] = np.array([0.1, 0.55, 0.])
            self.geom_data['points']['wing']['r_tip_le'] = np.array([0., 0.55, 0.])

            wing_primitive_names = list(spatial_rep.get_primitives(search_names=['Wing']).keys())
            wing = LiftingSurface(name='Wing', spatial_representation=spatial_rep, primitive_names=right_wing_names)
            if debug_geom_flag:
                wing.plot()

            self.sys_rep.add_component(wing)

            self.geom_data['components']['wing'] = wing
            self.geom_data['primitive_names']['wing'] = wing_primitive_names

            self.geom_data['primitive_names']['right_wing'] = right_wing_names
            self.geom_data['primitive_names']['right_wing_bottom'] = right_wing_bottom_names
            self.geom_data['primitive_names']['right_wing_top'] = right_wing_top_names

            # self.geom_data['components']['wing_te'] = wing_te
            self.geom_data['components']['wing_oml'] = wing_oml
            self.geom_data['components']['wing_top'] = wing_top
            self.geom_data['components']['wing_bottom'] = wing_bottom

        # endregion

        # endregion
        return


    def setup_internal_wingbox_geometry(self,
                                        debug_geom_flag=False,
                                        force_reprojection=False):

        spatial_rep = self.sys_rep.spatial_representation

        r_tip_le = self.geom_data['points']['wing']['r_tip_le']
        r_tip_te = self.geom_data['points']['wing']['r_tip_te']
        root_te = self.geom_data['points']['wing']['root_te']
        root_le = self.geom_data['points']['wing']['root_le']

        wing_right_bottom = self.geom_data['components']['wing_bottom']
        wing_right_top = self.geom_data['components']['wing_top']

        structural_right_wing_names = []

        # projections for internal structure
        num_rib_pts = 20

        # We start with projecting the ribsribs_top
        tip_te = r_tip_te
        tip_le = r_tip_le

        root_03 = (97 * root_le + 3*root_te) / 100
        root_97 = (3 * root_le + 97 * root_te) / 100
        tip_03 = (97 * tip_le + 3 * tip_te) / 100
        tip_97 = (3 * tip_le + 97 * tip_te) / 100

        second_rib_03 = root_03 + np.array([0., 0.041, 0.])
        second_rib_97 = root_97 + np.array([0., 0.041, 0.])

        rib_projection_points_y = np.arange(second_rib_03[1], tip_03[1]-0.035, 0.03825)
        tip_rib_points_y = np.array([rib_projection_points_y[-1] + 0.0325, 0.55])
        all_spar_y_points = np.concatenate(([root_03[1]], rib_projection_points_y, tip_rib_points_y))

        f_spar_projection_points = np.vstack([root_03[0]*np.ones_like(all_spar_y_points), all_spar_y_points, np.zeros_like(all_spar_y_points)]).T
        r_spar_projection_points = np.vstack([root_97[0]*np.ones_like(all_spar_y_points), all_spar_y_points, np.zeros_like(all_spar_y_points)]).T

        # f_spar_projection_points = np.linspace(root_03, tip_03, num_ribs)
        # r_spar_projection_points = np.linspace(root_97, tip_97, num_ribs)

        # ribs cover entire airfoil shape
        # f_spar_projection_points = np.linspace(root_le, tip_le, num_ribs)
        # r_spar_projection_points = np.linspace(root_te, tip_te, num_ribs)

        rib_projection_points = np.linspace(f_spar_projection_points, r_spar_projection_points, num_rib_pts)

        ribs_top = wing_right_top.project(rib_projection_points, direction=[0., 0., 1.], plot=debug_geom_flag,
                                         grid_search_n=100, force_reprojection=force_reprojection)
        ribs_bottom = wing_right_bottom.project(rib_projection_points, direction=[0., 0., 1.], plot=debug_geom_flag,
                                               grid_search_n=100, force_reprojection=force_reprojection)

        ribs_top_val = ribs_top.value
        ribs_bottom_val = ribs_bottom.value

        # make surface panels at root and tip
        n_cp = (num_rib_pts, 2)
        order = (2,)

        surface_dict = {}

        root_rib_top_points = ribs_top.value[:, (0, 1), :]
        root_rib_top_points[:, 1, :] = root_rib_top_points[:, 0, :] + np.vstack(num_rib_pts*[np.array([0., 0.012345, 0.])])  # end of the little root top surface (12.345 mm from root)
        t_panel_bspline = bsf.fit_bspline(root_rib_top_points, num_control_points=n_cp, order=order)
        t_panel = SystemPrimitive('root_t_panel', t_panel_bspline)
        surface_dict[t_panel.name] = t_panel
        structural_right_wing_names.append(t_panel.name)

        root_rib_bottom_points = ribs_bottom.value[:, (0, 1), :]
        root_rib_bottom_points[:, 1, :] = root_rib_bottom_points[:, 0, :] + np.vstack(num_rib_pts*[np.array([0., 0.012345, 0.])])  # end of the little root bottom surface (12.345 mm from root)
        t_panel_bspline = bsf.fit_bspline(root_rib_bottom_points, num_control_points=n_cp, order=order)
        t_panel = SystemPrimitive('root_b_panel', t_panel_bspline)
        surface_dict[t_panel.name] = t_panel
        structural_right_wing_names.append(t_panel.name)

        tip_rib_top_points = ribs_top.value[:, (-2, -1), :]
        t_panel_bspline = bsf.fit_bspline(tip_rib_top_points, num_control_points=n_cp, order=order)
        t_panel = SystemPrimitive('tip_t_panel', t_panel_bspline)
        surface_dict[t_panel.name] = t_panel
        structural_right_wing_names.append(t_panel.name)

        tip_rib_bottom_points = ribs_bottom.value[:, (-2, -1), :]
        b_panel_bspline = bsf.fit_bspline(tip_rib_bottom_points, num_control_points=n_cp, order=order)
        b_panel = SystemPrimitive('tip_b_panel', b_panel_bspline)
        surface_dict[b_panel.name] = b_panel
        structural_right_wing_names.append(b_panel.name)
        
        # make ribs (separate parts for 0%-20%, 20%-80% and 80%-100% of chord so the intersections match with the stiffener going through)
        n_cp_rib = (num_rib_pts, 2)
        order_rib = (2,)

        for i in range(all_spar_y_points.shape[0]):
            # first the bottom half of the rib
            rib_points = np.zeros((num_rib_pts, 2, 3))
            rib_points[:, 0, :] = ribs_bottom.value[:, i, :]
            rib_points[:, 1, :] = ribs_bottom.value[:, i, :]
            rib_points[:, 1, 2] = 0.
            rib_bspline = bsf.fit_bspline(rib_points, num_control_points=n_cp_rib, order=order_rib)
            rib = SystemPrimitive('b_rib_' + str(i), rib_bspline)
            spatial_rep.primitives[rib.name] = rib
            structural_right_wing_names.append(rib.name)

            # then the top half of the rib
            rib_points = np.zeros((num_rib_pts, 2, 3))
            rib_points[:, 0, :] = ribs_top.value[:, i, :]
            rib_points[:, 1, :] = ribs_top.value[:, i, :]
            rib_points[:, 1, 2] = 0.
            rib_bspline = bsf.fit_bspline(rib_points, num_control_points=n_cp_rib, order=order_rib)
            rib = SystemPrimitive('t_rib_' + str(i), rib_bspline)
            spatial_rep.primitives[rib.name] = rib
            structural_right_wing_names.append(rib.name)

        # make the central stiffener plate from 20% to 80% chord
        n_cp = (2, 2)
        order = (2,)

        for i in range(all_spar_y_points.shape[0]-1):
            stiffener_points = np.zeros((2, 2, 3))  # indices: 0 -> top, bottom of spar; 1 -> inboard, outboard rib location; 2 -> coordinate
            ribs_top_inp = ribs_top.value[0, (i, i + 1), :]
            ribs_bot_inp = ribs_bottom.value[0, (i, i + 1), :]
            ribs_top_inp[:, 0] = 0.02
            ribs_top_inp[:, 2] = 0.
            ribs_bot_inp[:, 0] = 0.08
            ribs_bot_inp[:, 2] = 0.            

            stiffener_points[0, :, :] = ribs_top_inp
            stiffener_points[1, :, :] = ribs_bot_inp

            # stiffener_points[0, 0, :] = (4*root_le + root_te)/5
            # stiffener_points[1, 0, :] = (root_le + 4*root_te)/5
            # stiffener_points[0, 1, :] = (4*r_tip_le + r_tip_te)/5
            # stiffener_points[1, 1, :] = (r_tip_le + 4*r_tip_te)/5

            stiffener_bspline = bsf.fit_bspline(stiffener_points, num_control_points=n_cp, order=order)
            stiffener = SystemPrimitive('stiffener_plate_{}'.format(i), stiffener_bspline)
            spatial_rep.primitives[stiffener.name] = stiffener
            structural_right_wing_names.append(stiffener.name)

        # make the front and rear spars
        spar_idxs = 0
        # we treat the section between the first and second ribs separately, since there's a surface intersection that we need to take into account
        f_spar_points_root = np.zeros((2, 2, 3))
        f_spar_points_root[0, :, :] = root_rib_top_points[0, :, :]
        f_spar_points_root[1, :, :] = root_rib_bottom_points[0, :, :]
        f_spar_bspline = bsf.fit_bspline(f_spar_points_root, num_control_points=n_cp, order=order)
        f_spar = SystemPrimitive('f_spar_{}'.format(spar_idxs), f_spar_bspline)
        spatial_rep.primitives[f_spar.name] = f_spar
        structural_right_wing_names.append(f_spar.name)

        r_spar_points_root = np.zeros((2, 2, 3))
        r_spar_points_root[0, :, :] = root_rib_top_points[-1, :, :]
        r_spar_points_root[1, :, :] = root_rib_bottom_points[-1, :, :]
        r_spar_bspline = bsf.fit_bspline(r_spar_points_root, num_control_points=n_cp, order=order)
        r_spar = SystemPrimitive('r_spar_{}'.format(spar_idxs), r_spar_bspline)
        spatial_rep.primitives[r_spar.name] = r_spar
        structural_right_wing_names.append(r_spar.name)

        spar_idxs += 1


        for i in range(all_spar_y_points.shape[0]-1):
            f_spar_points = np.zeros((2, 2, 3))
            f_spar_points[0, :, :] = ribs_top.value[0, (i, i + 1), :]
            f_spar_points[1, :, :] = ribs_bottom.value[0, (i, i + 1), :]
            if i == 0:
                # change the inboard rib location to the place where the inboard surface ends
                f_spar_points[:, 0, :] = f_spar_points_root[:, 1, :]
            f_spar_bspline = bsf.fit_bspline(f_spar_points, num_control_points=n_cp, order=order)
            f_spar = SystemPrimitive('f_spar_{}'.format(spar_idxs), f_spar_bspline)
            spatial_rep.primitives[f_spar.name] = f_spar
            structural_right_wing_names.append(f_spar.name)

            r_spar_points = np.zeros((2, 2, 3))
            r_spar_points[0, :, :] = ribs_top.value[-1, (i, i + 1), :]
            r_spar_points[1, :, :] = ribs_bottom.value[-1, (i, i + 1), :]
            if i == 0:
                # change the inboard rib location to the place where the inboard surface ends
                r_spar_points[:, 0, :] = r_spar_points_root[:, 1, :]
            r_spar_bspline = bsf.fit_bspline(r_spar_points, num_control_points=n_cp, order=order)
            r_spar = SystemPrimitive('r_spar_{}'.format(spar_idxs), r_spar_bspline)
            spatial_rep.primitives[r_spar.name] = r_spar
            structural_right_wing_names.append(r_spar.name)

            spar_idxs += 1

        # for i in range(num_ribs - 1):
        #     t_panel_points = ribs_top.value[:, (i, i + 1), :]
        #     t_panel_bspline = bsf.fit_bspline(t_panel_points, num_control_points=n_cp, order=order)
        #     t_panel = SystemPrimitive('t_panel_' + str(i), t_panel_bspline)
        #     surface_dict[t_panel.name] = t_panel
        #     structural_right_wing_names.append(t_panel.name)

        #     b_panel_points = ribs_bottom.value[:, (i, i + 1), :]
        #     b_panel_bspline = bsf.fit_bspline(b_panel_points, num_control_points=n_cp, order=order)
        #     b_panel = SystemPrimitive('b_panel_' + str(i), b_panel_bspline)
        #     surface_dict[b_panel.name] = b_panel
        #     structural_right_wing_names.append(b_panel.name)

        surface_dict.update(spatial_rep.primitives)
        spatial_rep.primitives = surface_dict
        self.geom_data['primitive_names']['structural_right_wing_names'] = structural_right_wing_names
        return

    def setup_index_functions(self):
        right_wing_names = self.geom_data['primitive_names']['right_wing']

        # wing force function
        num = 12
        u, v = np.meshgrid(np.linspace(0, 1, num), np.linspace(0, 1, num))
        u = np.array(u).flatten()
        v = np.array(v).flatten()
        points = np.vstack((u, v)).T
        space_f = IDWFunctionSpace(name='force_base_space', points=points, order=1,
                                   coefficients_shape=(points.shape[0],))
        wing_force = index_functions(right_wing_names, 'wing_force', space_f, 3)
        self.functions['wing_force'] = wing_force

        # wing cp function
        space_cp = lg.BSplineSpace(name='cp_base_space',
                                   order=(2, 4),
                                   control_points_shape=(2, 10))
        wing_cp = index_functions(right_wing_names, 'wing_cp', space_cp, 1)
        self.functions['wing_cp'] = wing_cp

        # wing oml geometry function:

        coefficients = {}
        geo_space = lg.BSplineSpace(name='geo_base_space',
                                    order=(4,4),
                                    control_points_shape=(15,15))
        wing_oml_geo = index_functions(right_wing_names, 'wing_oml_geo', geo_space, 3)
        spatial_rep = self.sys_rep.spatial_representation
        for name in right_wing_names:
            primitive = spatial_rep.get_primitives([name])[name].geometry_primitive
            coefficients[name] = m3l.Variable(name = name + '_geo_coefficients', shape = primitive.control_points.shape, value = primitive.control_points)

        wing_oml_geo.coefficients = coefficients
        self.functions['wing_geo'] = wing_oml_geo

        return


    def oml_mesh(self,
                 include_wing_flag=False,
                 grid_num_u=10, grid_num_v=10,
                 debug_geom_flag=False, force_reprojection=False):

        spatial_rep = self.sys_rep.spatial_representation

        # region Wing
        if include_wing_flag:
            # wing_component = self.geom_data['components']['wing']
            # wing_te_component = self.geom_data['components']['wing_te']

            # region OML nodes (parametric)
            # Right Wing
            right_wing_oml_para_coords = []
            for name in self.geom_data['primitive_names']['right_wing']:
                for u in np.linspace(0, 1, grid_num_u):
                    for v in np.linspace(0, 1, grid_num_v):
                        right_wing_oml_para_coords.append((name, np.array([u, v]).reshape((1, 2))))
            right_oml_geo_nodes = spatial_rep.evaluate_parametric(right_wing_oml_para_coords)
            right_wing_oml_geo_name = 'right_wing_oml_geo'
            self.sys_rep.add_output(right_wing_oml_geo_name, right_oml_geo_nodes)
            if debug_geom_flag:
                spatial_rep.plot_meshes([right_oml_geo_nodes])
            self.mesh_data['oml']['oml_geo_nodes']['right_wing'] = right_oml_geo_nodes
            self.mesh_data['oml']['oml_para_nodes']['right_wing'] = right_wing_oml_para_coords
            self.mesh_data['oml']['mesh_name']['right_wing'] = right_wing_oml_geo_name
            # endregion
        # endregion
        return


    def vlm_meshes(
            self,
            include_wing_flag=False, num_wing_spanwise_vlm=21, num_wing_chordwise_vlm=5,
            include_htail_flag=False, num_htail_spanwise_vlm=21, num_htail_chordwise_vlm=5,
            debug_geom_flag=False, visualize_flag=False,
            force_reprojection=False, ml=False
    ):
        spatial_rep = self.sys_rep.spatial_representation

        # region Wing
        if include_wing_flag:

            wing_component = self.geom_data['components']['wing']
            # wing_te_component = self.geom_data['components']['wing_te']

            # le_offset = np.array([-10, 0, 0])

            r_tip_le = self.geom_data['points']['wing']['r_tip_le']
            r_tip_te = self.geom_data['points']['wing']['r_tip_te']
            root_le = self.geom_data['points']['wing']['root_le']
            root_te = self.geom_data['points']['wing']['root_te']

            leading_edge_points = np.linspace(root_le, r_tip_le, num_wing_spanwise_vlm)
            trailing_edge_points = np.linspace(root_te, r_tip_te, num_wing_spanwise_vlm)

            leading_edge = wing_component.project(leading_edge_points, direction=np.array([-1., 0., 0.]),
                                                  plot=debug_geom_flag, force_reprojection=force_reprojection)
            trailing_edge = wing_component.project(trailing_edge_points, direction=np.array([1., 0., 0.]),
                                                      plot=debug_geom_flag, force_reprojection=force_reprojection)

            # Chord Surface
            wing_chord_surface = am.linspace(leading_edge, trailing_edge, num_wing_chordwise_vlm)
            if debug_geom_flag:
                spatial_rep.plot_meshes([wing_chord_surface])

            # Upper and lower surface
            wing_upper_surface_wireframe = wing_component.project(wing_chord_surface.value,
                                                            direction=np.array([0., 0., 1.]), grid_search_n=25,
                                                            plot=debug_geom_flag, max_iterations=200,
                                                            force_reprojection=force_reprojection)
            wing_lower_surface_wireframe = wing_component.project(wing_chord_surface.value,
                                                            direction=np.array([0., 0., -1.]), grid_search_n=25,
                                                            plot=debug_geom_flag, max_iterations=200,
                                                            force_reprojection=force_reprojection)

            # Chamber surface
            wing_camber_surface = am.linspace(wing_upper_surface_wireframe, wing_lower_surface_wireframe, 1)
            wing_vlm_mesh_name = 'wing_vlm_mesh'
            self.sys_rep.add_output(wing_vlm_mesh_name, wing_camber_surface)
            if debug_geom_flag:
                spatial_rep.plot_meshes([wing_camber_surface])
            self.mesh_data['vlm']['chamber_surface']['wing'] = wing_camber_surface
            self.mesh_data['vlm']['chamber_surface']['wing_undef'] = wing_camber_surface  # Added to include undeformed mesh in VLM solver
            self.mesh_data['vlm']['mesh_name']['wing'] = wing_vlm_mesh_name

            # Chord distribution
            self.sys_rep.add_output(name='wing_chord_distribution',
                               quantity=am.norm(leading_edge - trailing_edge))

        # endregion
        if visualize_flag:
            if include_wing_flag and not include_htail_flag :  # Wing-Only
                spatial_rep.plot_meshes([wing_camber_surface])
            elif include_htail_flag and not include_wing_flag:
                raise NotImplementedError
            elif include_wing_flag and include_htail_flag :
                spatial_rep.plot_meshes([wing_camber_surface, htail_camber_surface], primitives=self.geom_data['primitive_names']['wing'] + self.geom_data['primitive_names']['htail'])
        return

    def beam_mesh(self,
                  include_wing_flag=False, num_wing_beam_nodes=21,
                  beam_axis_location=0.35, front_spar_location=0.25, rear_spar_location=0.75,
                  height_computation_location=0.25,
                  debug_geom_flag=False, visualize_flag=False,
                  force_reprojection=True
                  ):

        spatial_rep = self.sys_rep.spatial_representation

        if include_wing_flag:

            wing_component = self.geom_data['components']['wing']
            wing_te_component = self.geom_data['components']['wing_te']

            r_tip_le = self.geom_data['points']['wing']['r_tip_le']
            l_tip_le = self.geom_data['points']['wing']['l_tip_le']
            r_tip_te = self.geom_data['points']['wing']['r_tip_te']
            root_te = self.geom_data['points']['wing']['root_te']
            l_tip_te = self.geom_data['points']['wing']['l_tip_te']

            le_offset = np.array([-10, 0, 0])

            leading_edge_points = np.linspace(r_tip_le, l_tip_le, num_wing_beam_nodes)
            trailing_edge_points = np.vstack((np.linspace(r_tip_te, root_te, int(num_wing_beam_nodes / 2) + 1),
                                              np.linspace(root_te, l_tip_te, int(num_wing_beam_nodes / 2) + 1)[1:,
                                              :]))

            wing_leading_edge = wing_component.project(leading_edge_points + le_offset, direction=np.array([0., 0., -1]),
                                                  plot=debug_geom_flag, force_reprojection=force_reprojection)
            wing_trailing_edge = wing_te_component.project(trailing_edge_points, direction=np.array([1., 0., 0.]),
                                                      plot=debug_geom_flag, force_reprojection=force_reprojection)

            # Wing beam nodes is located at 35% from the leading edge
            wing_beam = am.linear_combination(wing_leading_edge, wing_trailing_edge, 1,
                                              start_weights=np.ones((num_wing_beam_nodes,)) * (1-beam_axis_location),
                                              stop_weights=np.ones((num_wing_beam_nodes,)) * beam_axis_location)
            if debug_geom_flag:
                spatial_rep.plot_meshes([wing_beam])

            # Get the width of the beam
            front_spar = am.linear_combination(
                wing_leading_edge, wing_trailing_edge, 1,
                                              start_weights=np.ones((num_wing_beam_nodes,)) * (1-front_spar_location),
                                              stop_weights=np.ones((num_wing_beam_nodes,)) * front_spar_location
            )
            if debug_geom_flag:
                spatial_rep.plot_meshes([front_spar])

            rear_spar = am.linear_combination(
                wing_leading_edge, wing_trailing_edge, 1,
                start_weights=np.ones((num_wing_beam_nodes,)) * (1-rear_spar_location),
                stop_weights=np.ones((num_wing_beam_nodes,)) * rear_spar_location
            )
            if debug_geom_flag:
                spatial_rep.plot_meshes([rear_spar])

            width = am.norm((front_spar.reshape((-1, 3)) - rear_spar.reshape((-1, 3))) * 1)

            # Get the height of the beam
            offset = np.array([0, 0, 0.5])
            height_location = am.linear_combination(
                wing_leading_edge, wing_trailing_edge, 1,
                start_weights=np.ones((num_wing_beam_nodes,)) * (1 - height_computation_location),
                stop_weights=np.ones((num_wing_beam_nodes,)) * height_computation_location
            )
            if debug_geom_flag:
                spatial_rep.plot_meshes([height_location])
            top = wing_component.project(height_location.value + offset, direction=np.array([0., 0., -1.]), plot=debug_geom_flag)
            bot = wing_component.project(height_location.value - offset, direction=np.array([0., 0., 1.]), plot=debug_geom_flag)
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
                spatial_rep.plot_meshes([wing_beam, front_spar, rear_spar], primitives=self.geom_data['primitive_names']['wing'])
        return