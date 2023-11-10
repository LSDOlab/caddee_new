import m3l
import numpy as np
from dataclasses import dataclass, field
import lsdo_geo as lg
from scipy.interpolate import interp1d
from typing import List


@dataclass
class RotorMeshes:
    """
    Data class for rotor meshes
    """
    thrust_origin : m3l.Variable
    thrust_vector : m3l.Variable
    radius : m3l.Variable
    in_plane_1 : m3l.Variable
    in_plane_2 : m3l.Variable
    disk_mesh : m3l.Variable = None
    chord_profile : m3l.Variable = None
    twist_profile : m3l.Variable = None
    vlm_meshes : List[m3l.Variable] = field(default_factory=list)


@dataclass
class BladeParameters:
    """
    Data class for specifying blade geometric paramters
    """
    blade_component : lg.BSplineSubSet
    point_on_leading_edge : np.ndarray
    num_spanwise_vlm : int = None
    num_chordwise_vlm : int = None


@dataclass
class SimpleBoxBeamMesh:
    """
    Data class for box beam meshes
    """
    height : m3l.Variable
    width : m3l.Variable
    beam_nodes : m3l.Variable



def make_rotor_mesh(
        geometry : lg.Geometry,
        num_radial : int, 
        disk_component : lg.BSplineSubSet,
        origin : np.ndarray, 
        y1 : np.ndarray,
        y2 : np.ndarray,
        z1 : np.ndarray,
        z2 : np.ndarray,
        blade_geometry_parameters : List[BladeParameters] = [],
        num_tangential : int = 25,
        norm_hub_radius : float = 0.2,
        create_disk_mesh = False,
        plot : bool = False,
        grid_search_density_parameter : int = 25
) -> RotorMeshes: 
    """
    Helper function to automate generation of rotor meshes
    """
    # Disk origin
    disk_origin = geometry.evaluate(
        disk_component.project(origin, 
                               grid_search_density_parameter=grid_search_density_parameter), 
                                plot=plot)
    
    # In-plane 1
    y11 = geometry.evaluate(disk_component.project(y1, plot=plot))
    y12 = geometry.evaluate(disk_component.project(y2, plot=plot))
    disk_in_plane_y = y11 - y12

    # In-plane 2
    y21 = geometry.evaluate(disk_component.project(z1, plot=plot))
    y22 = geometry.evaluate(disk_component.project(z2, plot=plot))
    disk_in_plane_x = y21 - y22

    rotor_radius = m3l.norm(y12 - y11) / 2

    thrust_vector = m3l.cross(disk_in_plane_x, disk_in_plane_y)
    thrust_unit_vector = thrust_vector / m3l.norm(thrust_vector)

    rotor_mesh = RotorMeshes(
        thrust_origin=disk_origin,
        thrust_vector=thrust_unit_vector,
        radius=rotor_radius,
        in_plane_1=disk_in_plane_x,
        in_plane_2=disk_in_plane_y,
    )

    if create_disk_mesh:
        v1 = disk_in_plane_y / m3l.norm(disk_in_plane_y)
        v2 = disk_in_plane_x / m3l.norm(disk_in_plane_x)
        p = disk_origin
        
        if num_radial %2 != 0:
            raise ValueError(f"Odd number 'num_radial' not yet implemented. Must be an even number for now")
        
        radii = np.linspace(norm_hub_radius * rotor_radius.value, rotor_radius.value, num_radial)
        angles = np.linspace(0, 2*np.pi, num_tangential)
        
        cartesian = np.zeros((num_radial, num_tangential, 3))
        for i in range(num_radial):
            for j in range(num_tangential):
                # Equation of a circle (circular plane) in 3D space that does not have to be aligned with any 2 cartesian axes
                cartesian[i, j, :] = p.value + radii[i] * np.cos(angles[j]) * v1.value + radii[i] * np.sin(angles[j]) * v2.value
        
        disk_mesh = geometry.evaluate(disk_component.project(cartesian, plot=plot))
        rotor_mesh.disk_mesh=disk_mesh

    if blade_geometry_parameters:
        counter = 0
        for blade in blade_geometry_parameters:
            comp = blade.blade_component
            p_o_le = blade.point_on_leading_edge
            num_spanwise = blade.num_spanwise_vlm
            if num_spanwise %2 != 0:
                raise ValueError(f"Odd number for 'num_spanwise_vlm' not yet implemented. Must be even number for now")
            num_chordwise = blade.num_chordwise_vlm

            b_spline_patch = comp.project(p_o_le, plot=plot)[0][0]

            if counter == 0:

                # le_tip = blade.le_tip
                # le_hub = blade.le_hub
                # le_center = blade.le_center
                # te_center = blade.te_center

                # ec = (le_tip - le_hub) / 2 + le_hub
                # u = le_tip - ec
                # v = le_center- ec

                # half_ellipse = np.zeros((num_radial, 3))
                # angles = np.linspace(0, np.pi, num_radial)
                # for i in range(num_radial):
                #     alpha = 2 * angles[i]
                #     theta = 2 * alpha / num_radial
                #     # half_ellipse[i, :] = (ec + np.sin(alpha-theta) * u + np.sin(theta) * v) / np.sin(alpha)
                #     half_ellipse[i, :] = ec + np.cos(angles[i]) * u + np.sin(angles[i]) * v

                # # print(half_ellipse)
                # straight_line = np.linspace(le_hub, le_tip, num_radial)


                linspace_parametric = np.hstack((np.linspace(0, 0.55, int(num_radial/2)), np.linspace(0.7, 1, int(num_radial/2))))
                linspace_parametric_vlm = np.hstack((np.linspace(0, 0.55, int(num_spanwise/2)), np.linspace(0.7, 1, int(num_spanwise/2))))
                le_list = []
                te_list = []

                
                for i in range(num_radial):
                    le_list.append((b_spline_patch, np.array([[linspace_parametric[i], 1]])))
                    te_list.append((b_spline_patch, np.array([[linspace_parametric[i], 0]])))

                le = geometry.evaluate(le_list).reshape((-1, 3))
                te = geometry.evaluate(te_list).reshape((-1, 3))
                # le_minus_te = le-te
                le_minus_te = te-le

                normal_exp = m3l.expand(thrust_unit_vector, new_shape=(num_radial, 3), indices='i->ji')
                twist_profile = np.pi/2 - m3l.arccos(m3l.dot(normal_exp, le_minus_te, axis=1)/ m3l.norm(le_minus_te, axes=(1, )))
                chord_profile = m3l.norm(le_minus_te)

                # print(chord_profile)
                # print(twist_profile)

                rotor_mesh.chord_profile = chord_profile
                rotor_mesh.twist_profile = twist_profile

            if num_spanwise is not None:
                le_list_vlm = []
                te_list_vlm = []
                for i in range(num_spanwise):
                    le_list_vlm.append((b_spline_patch, np.array([[linspace_parametric_vlm[i], 1]])))
                    te_list_vlm.append((b_spline_patch, np.array([[linspace_parametric_vlm[i], 0]])))

                le_vlm = geometry.evaluate(le_list_vlm).reshape((-1, 3))
                te_vlm = geometry.evaluate(te_list_vlm).reshape((-1, 3))

                chord_surface = m3l.linspace(le_vlm, te_vlm, num_chordwise)# .reshape((-1, 3))
                if num_chordwise > 2:
                    upper_surface = geometry.evaluate(comp.project(chord_surface.value + thrust_vector.value, direction=thrust_unit_vector.value, plot=plot)).reshape((num_chordwise, num_spanwise, 3))
                    lower_surface = geometry.evaluate(comp.project(chord_surface.value - thrust_vector.value, direction=-1 * thrust_unit_vector.value, plot=plot)).reshape((num_chordwise, num_spanwise, 3))
                    camber_surface = m3l.linspace(upper_surface, lower_surface, 1)
                else: 
                    camber_surface = chord_surface.reshape((num_chordwise, num_spanwise, 3))

                if plot:
                    geometry.plot_meshes(meshes=camber_surface, mesh_plot_types=['wireframe'], mesh_opacity=1., mesh_color='#F5F0E6')


                rotor_mesh.vlm_meshes.append(camber_surface)

            counter += 1

            

    return rotor_mesh


def make_vlm_camber_mesh(
        geometry : lg.Geometry,
        wing_component : lg.BSplineSubSet,
        num_spanwise : int,
        num_chordwise : int, 
        le_right : np.ndarray,
        le_left : np.ndarray,
        te_right : np.ndarray,
        te_left : np.ndarray,
        le_center : np.ndarray = None,
        te_center : np.ndarray = None,
        plot: bool=False,
        grid_search_density_parameter : int = 50,
        le_interp : str = 'ellipse',
        te_interp : str = 'ellipse',
) -> m3l.Variable: 
    """
    Helper function to create a VLM camber mesh
    """
    if le_center is not None:
        x = np.array([le_left[0], le_center[0], le_right[0]])
        y = np.array([le_left[1], le_center[1], le_right[1]])
        z = np.array([le_left[2], le_center[2], le_right[2]])
        fz = interp1d(y, z, kind='linear')
        array_to_project = np.zeros((num_spanwise, 3))
        interp_y = np.linspace(y[0], y[2], num_spanwise)
        
        if le_interp == 'ellipse':
            # Parameters for ellipse
            h = le_right[0]
            b = h - le_center[0]
            a = le_right[1]

            array_to_project[:, 0] = -(b**2 * (1 - interp_y**2/a**2))**0.5 + h
            array_to_project[:, 1] = interp_y
            array_to_project[:, 2] = fz(interp_y)

        elif le_interp == 'linear':
            fx = interp1d(y, x, kind='linear')
            array_to_project[:, 0] = fx(interp_y)
            array_to_project[:, 1] = interp_y
            array_to_project[:, 2] = fz(interp_y)

        elif le_interp == 'quadratic':
            fx = interp1d(y, x, kind='quadratic')
            array_to_project[:, 0] = fx(interp_y)
            array_to_project[:, 1] = interp_y
            array_to_project[:, 2] = fz(interp_y)

        elif le_interp not in ['ellipse', 'linear', 'quadratic']:
            raise Exception(f"Unknown interpolation type '{le_interp}'. Available options are 'ellipse', 'linear', 'quadratic'.")
        else:
            raise NotImplementedError
        
        le = geometry.evaluate(wing_component.project(array_to_project, plot=plot, grid_search_density_parameter=grid_search_density_parameter)).reshape((-1, 3))
    
    else:
        le = geometry.evaluate(wing_component.project(np.linspace(le_left, le_right, num_spanwise), plot=plot, grid_search_density_parameter=grid_search_density_parameter)).reshape((-1, 3))


    if te_center is not None:
        x = np.array([te_left[0], te_center[0], te_right[0]])
        y = np.array([te_left[1], te_center[1], te_right[1]])
        z = np.array([te_left[2], te_center[2], te_right[2]])
        fz = interp1d(y, z, kind='linear')
        array_to_project = np.zeros((num_spanwise, 3))
        interp_y = np.linspace(y[0], y[2], num_spanwise)

        if te_interp == 'ellipse':
            # Parameters for ellipse
            h = te_right[0]
            b = h - te_center[0]
            a = te_right[1]

            array_to_project[:, 0] = (b**2 * (1 - interp_y**2/a**2))**0.5 + h
            array_to_project[:, 1] = interp_y
            array_to_project[:, 2] = fz(interp_y)

        elif te_interp == 'linear':
            fx = interp1d(y, x, kind='linear')
            array_to_project[:, 0] = fx(interp_y)
            array_to_project[:, 1] = interp_y
            array_to_project[:, 2] = fz(interp_y)

        elif te_interp == 'quadratic':
            fx = interp1d(y, x, kind='quadratic')
            array_to_project[:, 0] = fx(interp_y)
            array_to_project[:, 1] = interp_y
            array_to_project[:, 2] = fz(interp_y)

        elif le_interp not in ['ellipse', 'linear', 'quadratic']:
            raise Exception(f"Unknown interpolation type '{le_interp}'. Available options are 'ellipse', 'linear', 'quadratic'.")
        else:
            raise NotImplementedError
        
        te = geometry.evaluate(wing_component.project(array_to_project, plot=plot, grid_search_density_parameter=grid_search_density_parameter)).reshape((-1, 3))
    
    else:
        te = geometry.evaluate(wing_component.project(np.linspace(te_left, te_right, num_spanwise), plot=plot, grid_search_density_parameter=grid_search_density_parameter)).reshape((-1, 3))

    wing_chord_surface = m3l.linspace(le, te, num_chordwise)

    wing_upper_surface_wireframe_parametric = wing_component.project(wing_chord_surface.value + np.array([0., 0., 1.]), direction=np.array([0., 0., 1.]), grid_search_density_parameter=25, plot=plot)
    wing_lower_surface_wireframe_parametric = wing_component.project(wing_chord_surface.value - np.array([0., 0., 1.]), direction=np.array([0., 0., -1.]), grid_search_density_parameter=25, plot=plot)
    wing_upper_surface_wireframe = geometry.evaluate(wing_upper_surface_wireframe_parametric).reshape((num_chordwise, num_spanwise, 3))
    wing_lower_surface_wireframe = geometry.evaluate(wing_lower_surface_wireframe_parametric).reshape((num_chordwise, num_spanwise, 3))

    wing_camber_surface = m3l.linspace(wing_upper_surface_wireframe, wing_lower_surface_wireframe, 1)#.reshape((-1, 3))
    if plot:
        geometry.plot_meshes(meshes=wing_camber_surface, mesh_plot_types=['wireframe'], mesh_opacity=1., mesh_color='#F5F0E6')


    return wing_camber_surface


def make_1d_box_beam_mesh(
        geometry : lg.Geometry,
        wing_component : lg.BSplineSubSet,
        num_beam_nodes : int,
        le_right : np.ndarray,
        le_left : np.ndarray,
        te_right : np.ndarray,
        te_left : np.ndarray,
        beam_width : float,
        node_center : float,
        le_center : np.ndarray = None,
        te_center : np.ndarray = None,
        plot: bool=False,
        grid_search_density_parameter : int = 50,
        le_interp : str = 'ellipse',
        te_interp : str = 'ellipse',
) -> SimpleBoxBeamMesh: 
    """
    Helper function to create a simple 1-D box beam mesh with the beam node being 
    centered in the middle of the wing box       
    """

    if num_beam_nodes %2 == 0:
        raise ValueError("Number of beam nodes should be odd such that there is always a node at the center (of the fuselage)")

    if le_center is not None:
        x = np.array([le_left[0], le_center[0], le_right[0]])
        y = np.array([le_left[1], le_center[1], le_right[1]])
        z = np.array([le_left[2], le_center[2], le_right[2]])
        fz = interp1d(y, z, kind='linear')
        array_to_project = np.zeros((num_beam_nodes, 3))
        interp_y = np.linspace(y[0], y[2], num_beam_nodes)
        
        if le_interp == 'ellipse':
            # Parameters for ellipse
            h = le_right[0]
            b = h - le_center[0]
            a = le_right[1]

            array_to_project[:, 0] = -(b**2 * (1 - interp_y**2/a**2))**0.5 + h
            array_to_project[:, 1] = interp_y
            array_to_project[:, 2] = fz(interp_y)

        elif le_interp == 'linear':
            fx = interp1d(y, x, kind='linear')
            array_to_project[:, 0] = fx(interp_y)
            array_to_project[:, 1] = interp_y
            array_to_project[:, 2] = fz(interp_y)

        elif le_interp == 'quadratic':
            fx = interp1d(y, x, kind='quadratic')
            array_to_project[:, 0] = fx(interp_y)
            array_to_project[:, 1] = interp_y
            array_to_project[:, 2] = fz(interp_y)

        elif le_interp not in ['ellipse', 'linear', 'quadratic']:
            raise Exception(f"Unknown interpolation type '{le_interp}'. Available options are 'ellipse', 'linear', 'quadratic'.")
        else:
            raise NotImplementedError
        
        le = geometry.evaluate(wing_component.project(array_to_project, plot=plot, grid_search_density_parameter=grid_search_density_parameter)).reshape((-1, 3))
    
    else:
        le = geometry.evaluate(wing_component.project(np.linspace(le_left, le_right, num_beam_nodes), plot=plot, grid_search_density_parameter=grid_search_density_parameter)).reshape((-1, 3))


    if te_center is not None:
        x = np.array([te_left[0], te_center[0], te_right[0]])
        y = np.array([te_left[1], te_center[1], te_right[1]])
        z = np.array([te_left[2], te_center[2], te_right[2]])
        fz = interp1d(y, z, kind='linear')
        array_to_project = np.zeros((num_beam_nodes, 3))
        interp_y = np.linspace(y[0], y[2], num_beam_nodes)

        if te_interp == 'ellipse':
            # Parameters for ellipse
            h = te_right[0]
            b = h - te_center[0]
            a = te_right[1]

            array_to_project[:, 0] = (b**2 * (1 - interp_y**2/a**2))**0.5 + h
            array_to_project[:, 1] = interp_y
            array_to_project[:, 2] = fz(interp_y)

        elif te_interp == 'linear':
            fx = interp1d(y, x, kind='linear')
            array_to_project[:, 0] = fx(interp_y)
            array_to_project[:, 1] = interp_y
            array_to_project[:, 2] = fz(interp_y)

        elif te_interp == 'quadratic':
            fx = interp1d(y, x, kind='quadratic')
            array_to_project[:, 0] = fx(interp_y)
            array_to_project[:, 1] = interp_y
            array_to_project[:, 2] = fz(interp_y)

        elif le_interp not in ['ellipse', 'linear', 'quadratic']:
            raise Exception(f"Unknown interpolation type '{le_interp}'. Available options are 'ellipse', 'linear', 'quadratic'.")
        else:
            raise NotImplementedError
        
        te = geometry.evaluate(wing_component.project(array_to_project, plot=plot, grid_search_density_parameter=grid_search_density_parameter)).reshape((-1, 3))
    
    else:
        te = geometry.evaluate(wing_component.project(np.linspace(te_left, te_right, num_beam_nodes), plot=plot, grid_search_density_parameter=grid_search_density_parameter)).reshape((-1, 3))

    beam_nodes = m3l.linear_combination(le, te, 1, start_weights=np.ones((num_beam_nodes, ))*(1-node_center), stop_weights=np.ones((num_beam_nodes, ))*node_center).reshape((num_beam_nodes, 3))

    width = m3l.norm((le - te)*0.5)
    offset = np.array([0,0,0.5])
    top_parametric = wing_component.project(beam_nodes.value+offset, direction=np.array([0., 0., -1.]), plot=plot)
    bot_parametric = wing_component.project(beam_nodes.value-offset, direction=np.array([0., 0., 1.]), plot=plot)

    top = geometry.evaluate(top_parametric).reshape((num_beam_nodes, 3))
    bot = geometry.evaluate(bot_parametric).reshape((num_beam_nodes, 3))
    height = m3l.norm((top - bot)*1)

    if plot:
        geometry.plot_meshes(meshes=beam_nodes)# , mesh_plot_types=['surface'], mesh_opacity=1., mesh_color='#F5F0E6')

    box_beam_mesh = SimpleBoxBeamMesh(
        height=height,
        width=width,
        beam_nodes=beam_nodes,
    )

    return box_beam_mesh



   

