import numpy as np
ft2m = 0.3048

def visualize_ex_pav(
        caddee,
        sim,
        displacements,
        beam_mesh,
        web_t, 
        cap_t,
        width,
        height,
        rotor_origins,
        forces_index_function,
    ):
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=- Plotting parameters =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-:
    # Vedo
    import vedo
    show = True
    show_axes = False

    # params for index function
    grid_num = 10 # for index function compute grid
    vmin = None # min max values for colormap
    vmax = None # min max values for colormap
    cmap_index = 'coolwarm' # colormap for thickness

    # params for other
    basecolor = (115, 147, 179) # color of geometry
    displacement_color = (0, 0, 0) 
    thrust_vector_color = (250, 0, 0)  # color of rotor thrust vector
    thrust_vector_length = 0.7  # color of rotor thrust vector

    min_thickness = 0.000508 # max thickness for colormap
    max_thickness = 0.02 # max thickness for colormap
    cmap_thickness = 'coolwarm' # colormap for thickness

    min_z_displacement = -0.018 # min z displacement for colormap
    max_z_displacement = 0.1 # max z displacement for colormap
    cmap_displacement = 'copper'# colormap for displacement

    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=- Plotting parameters =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-:

    plotter = vedo.Plotter(offscreen= (not show),  shape = [dict(bottomleft=(0.0, 0.0), topright=(1.0, 1.0))])

    system_representation = caddee.system_representation
    spatial_rep = system_representation.spatial_representation
    geo_name = 'system_representation.system_configurations_model.design_geometry'
    geo_val = sim[geo_name]
    spatial_rep.update(geo_val)
    updated_primitives_names = []
    for primitive_name in spatial_rep.primitives.keys():
        if 'Wing' in primitive_name:
            continue
        updated_primitives_names.append(primitive_name)
    plotting_elements = spatial_rep.plot(
        show = 0,
        primitives = updated_primitives_names,
        color = basecolor,
        opacity = 0.3,
    )
    center_x = 4  # eyeballing center x coordinate of geometry
    center_z = 1  # eyeballing center z coordinate of geometry
    center_y = 0
    camera_settings = {
        'pos': (-6.4, 6.4, 8.8),
        # 'pos': (-26, -26, 30),
        'viewup': (0, 0, 1),
        # 'focalPoint': (center_x+8, 0+6, center_z-15)
        'focalPoint': (center_x, center_y-1, center_z-3.0)
    }

    # Lines between segments and spheres at the end of each segment
    for disp_condition, displ in enumerate(displacements):
        displ[:,2] = -displ[:,2]
        # print(np.max(displ[:,2]),np.min(displ[:,2]) )
        for segment_number in range(displ.shape[0]-1):
            # tuning parameters
            sphere_radius =  0.03
            linewidth = 5
            color = displacement_color
            displacement_factor = 10

            # build line and dot at the end
            start = displacement_factor*displ[segment_number, :] + beam_mesh[segment_number, :]
            end = displacement_factor*displ[segment_number+1, :] + beam_mesh[segment_number+1, :]
            if end[1] < 0:
                continue

            import matplotlib
            cmap = matplotlib.colormaps[cmap_displacement]
            avg_disp = (displ[segment_number, 2] + displ[segment_number+1, 2])/2
            c = cmap((avg_disp - min_z_displacement) / (max_z_displacement - min_z_displacement))
            line = vedo.shapes.Line(start, end, lw=linewidth, c=(c[0], c[1], c[2]))
            sphere = vedo.shapes.Sphere(end, r=sphere_radius, c=color)
            plotting_elements.append(line)
            # plotting_elements.append(sphere)

            if segment_number == 0:
                sphere = vedo.shapes.Sphere(start, r=sphere_radius, c=basecolor)
                plotting_elements.append(line)
                # plotting_elements.append(sphere)

            if disp_condition > 0:
                continue
            # drawing the f***ing rectangle:
            width_segment = (width[segment_number] + width[segment_number+1])/2
            height_segment = (height[segment_number] + height[segment_number+1])/2

            rectangle_top_right = ([width_segment/2,   height_segment/2, 0])
            rectangle_bot_right = ([width_segment/2,  -height_segment/2, 0])
            rectangle_top_left =  ([-width_segment/2,  height_segment/2, 0])
            rectangle_bot_left =  ([-width_segment/2, -height_segment/2, 0])
            old_points = [rectangle_top_right, rectangle_top_left, rectangle_bot_left, rectangle_bot_right]
            
            # Rotation to rotate matrix to normal
            middle_point = (end+start)/2
            normal = end - start
            rotmat = rotation_matrix_from_z_axis_to_vector(normal)

            # Rotation to align the rectangle with xy plane on normal plane
            vec = [rectangle_top_right[i]-rectangle_top_left[i] for i in range(len(rectangle_top_left))]
            normal_plane = np.array([normal[0], normal[1], normal[2], 0])
            z_plane = np.array([0, 0, 1, 0])
            u, c = get_plane_plane_intersection(normal_plane, z_plane)
            angle_rot = angle(vec, u+c)
            rotmat2 = rotation_matrix_on_plane(normal, -angle_rot)
            new_points = []
            for old_point in old_points:
                new_point = rotmat2@(rotmat@old_point) + middle_point
                new_points.append(new_point)
                # sphere = vedo.shapes.Sphere(new_point, r=linewidth/100, c=basecolor)
                # plotting_elements.append(sphere)
            
            for i in range(len(new_points)):
                start = new_points[i]
                
                if i == (len(new_points)-1):
                    end = new_points[0]
                else:
                    end = new_points[i+1]
                
                # thicknesses:
                web_thickness = web_t[segment_number]
                cap_thickness = cap_t[segment_number]
                

                if (i == 0) or (i == 2):
                    thickness = web_thickness
                else:
                    thickness = cap_thickness

                # c = interpolate_color(min_thickness, max_thickness, thickness_min_color, thickness_max_color, thickness)
                
                import matplotlib
                cmap = matplotlib.colormaps[cmap_thickness]
                c = cmap((thickness - min_thickness) / (max_thickness - min_thickness))
                line = vedo.shapes.Line(start, end, lw=linewidth, c=(c[0], c[1], c[2]))
            
                plotting_elements.append(line)


    # arrows
    for rotor_origin in rotor_origins:
        arrow = vedo.Arrow(
            start_pt = rotor_origin*ft2m,
            end_pt = rotor_origin*ft2m + np.array([0, 0, thrust_vector_length]),
            c = thrust_vector_color,
        )
        plotting_elements.append(arrow)

    # ###############################################:wing forces###############################################:
    surface_names  = list(forces_index_function.coefficients.keys())
    color  = color
    names_to_save = []
    index = 0
    from csdl import GraphRepresentation
    rep = sim.system_graph.rep
    if isinstance(rep, GraphRepresentation):
        surface_names_temp = set(surface_names)
        surface_names = []
        for unpromoted_name in rep.unpromoted_to_promoted:
            remove_surf_names = set()
            for name in surface_names_temp:
                if forces_index_function.coefficients[name].name  in unpromoted_name:
                    names_to_save.append(unpromoted_name)
                    surface_names.append(name)
    transfer_para_mesh = []
    surface_names_to_indices_dict = {}
    end_index = 0
    for name in surface_names:
        start_index = end_index
        for u in np.linspace(0,1,grid_num):
            for v in np.linspace(0,1,grid_num):
                transfer_para_mesh.append((name, np.array([u,v]).reshape((1,2))))
                end_index = end_index + 1
        surface_names_to_indices_dict[name] = (start_index,end_index)
    coefficients = {}
    for i, name in enumerate(surface_names):
        sim_name = names_to_save[i]
        coefficients[name] = sim[sim_name]

    evaluated_states = forces_index_function.compute(transfer_para_mesh, coefficients)
    num_states_per_point = evaluated_states.shape[-1]

    system_representation = caddee.system_representation
    spatial_rep = system_representation.spatial_representation
    locations = spatial_rep.evaluate_parametric(transfer_para_mesh).value
    # x = locations[:,0]
    # y = locations[:,1]
    # z = locations[:,2]

    # v = np.linalg.norm(evaluated_states, axis=1)
    v = (evaluated_states)
    # print(v)

    if num_states_per_point != 1:
        print('states per locoation is not a scalar. taking norm...')
        v = np.linalg.norm(evaluated_states, axis=1)

    # print(v.shape, index_function.name)
    if vmin is None:
        min_v = np.min(v)
        vmin = min_v
    else:
        min_v = vmin
    if vmax is None:
        max_v = np.max(v)
        vmax = max_v
    else:
        max_v = vmax
    remove_primitives = []
    for surf_num, (name, (start_index, end_index) )in enumerate(surface_names_to_indices_dict.items()):
        
        if name in remove_primitives:
            continue
        # if surf_num > 0:
        #     break
        color_map = []
        v_reshaped = v[start_index:end_index].reshape((grid_num,grid_num))
        vertices = []
        faces = []
        arrows = []
        reshaped_plot_points = locations[start_index:end_index].reshape((grid_num,grid_num,3))

        if np.max(reshaped_plot_points[:,:,1]) > 0:
            continue
        for i in range(grid_num):
            for ii in range(grid_num):
                vertex = tuple(reshaped_plot_points[i,ii,:])
                vertices.append(vertex)

                if i != 0 and ii != 0:
                    num_pts = grid_num
                    face = tuple((
                        (i-1)*num_pts+(ii-1),
                        (i-1)*num_pts+(ii),
                        (i)*num_pts+(ii),
                        (i)*num_pts+(ii-1),
                    ))

                    faces.append(face)

                    # print(face, vertex)
        # print(len(vertices), len(faces))
                color_map.append(v_reshaped[i,ii])

        mesh = vedo.Mesh([vertices, faces], c = color).opacity(1.0)
        mesh.cmap(cmap_index, color_map, vmin = min_v, vmax = max_v)

            # mesh.cmap('coolwarm', color_map)
        # mesh.point_size(3)
        plotting_elements.append(mesh)

    # exit()
    # plotter.show(plotting_elements, interactive=False, camera = camera_settings)
    plotter.show(plotting_elements,camera = camera_settings, interactive=show,  axes = show_axes)
    # image_data = plotter.screenshot(asarray=True)
    # num_y = image_data.shape[0]
    # num_x = image_data.shape[1]
    # image_data = image_data[num_y - int(0.7*num_y):num_y,0:int(num_x)]

    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(15, 10))
    gs = fig.add_gridspec(
        nrows=1,
        ncols=5,
        wspace=0.4,
        hspace=0.4,
    )
    plt.rcParams.update({'font.size': 25})

    # Colormap for displacement
    ax_subplot = fig.add_subplot(gs[0])
    plt.colorbar(
        matplotlib.cm.ScalarMappable(
            norm=matplotlib.colors.Normalize(vmin=min_z_displacement, vmax=max_z_displacement), cmap=cmap_displacement),
            cax = ax_subplot)
    ax_subplot.set_title('Displacement\n(z direction)')

    # Colormap for thickness
    ax_subplot = fig.add_subplot(gs[2])
    plt.colorbar(
        matplotlib.cm.ScalarMappable(
            norm=matplotlib.colors.Normalize(vmin=min_thickness, vmax=max_thickness ), cmap=cmap_thickness),
            cax = ax_subplot)
    ax_subplot.set_title('Thickness')
    
    # Colormap for Forces
    ax_subplot = fig.add_subplot(gs[4])
    plt.colorbar(
        matplotlib.cm.ScalarMappable(
            norm=matplotlib.colors.Normalize(vmin=vmin, vmax=vmax ), cmap=cmap_index),
            cax = ax_subplot)
    ax_subplot.set_title('Forces')
    
    plt.show()


def rotation_matrix_from_z_axis_to_vector(vector):
    from scipy.spatial.transform import Rotation
    # Normalize the input vector
    vector = np.array(vector) / np.linalg.norm(vector)
    
    # Find the axis of rotation using cross product
    axis = np.cross([0, 0, 1], vector)
    
    # Find the angle of rotation using the dot product
    angle = np.arccos(np.dot([0, 0, 1], vector))
    
    # Create a rotation object using the axis and angle
    r = Rotation.from_rotvec(angle * axis)
    
    # Get the rotation matrix
    rotation_matrix = r.as_matrix()
    
    return rotation_matrix



def rotation_matrix_on_plane(normal, theta):
    # Normalize the normal vector
    normal = np.array(normal) / np.linalg.norm(normal)
    
    # Compute the skew-symmetric matrix of the normal vector
    skew_matrix = np.array([[0, -normal[2], normal[1]],
                            [normal[2], 0, -normal[0]],
                            [-normal[1], normal[0], 0]])
    
    # Compute the rotation matrix using the Rodrigues' formula
    rotation_matrix = np.eye(3) + np.sin(theta) * skew_matrix + (1 - np.cos(theta)) * np.dot(skew_matrix, skew_matrix)
    
    return rotation_matrix

def angle(v1, v2):
    # v1 is your firsr vector
    # v2 is your second vector
    angle = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
    return angle

def norm2(X):
	return np.sqrt(np.sum(X ** 2))

def normalized(X):
	return X / norm2(X)

'''
Returns the intersection, a line, between the plane A and B
  - A and B are planes equations, such as A0 * x + A1 * y + A2 * z + A3 = 0
  - The line is returned as (U, V), where any point of the line is t * U + C, for all values of t
  - U is a normalized vector
  - C is the line origin, with the triangle (Ao, Bo, C) is orthogonal to the plane A and B,
    with Ao and Bo being the origin of plane A an B
  - If A and B are parallel, a np.linalg.LinAlgError exception is raised
'''
def get_plane_plane_intersection(A, B):
	U = normalized(np.cross(A[:-1], B[:-1]))
	M = np.array((A[:-1], B[:-1], U))
	X = np.array((-A[-1], -B[-1], 0.))
	return U, np.linalg.solve(M, X)	


def interpolate_color(min_value, max_value, min_color, max_color, value):
    # Make sure the given value is within the range [min_value, max_value]
    value = max(min_value, min(max_value, value))
    
    # Calculate the ratio of the value within the range [0, 1]
    ratio = (value - min_value) / (max_value - min_value)
    
    # Interpolate the RGB values
    red = int(min_color[0] + ratio * (max_color[0] - min_color[0]))
    green = int(min_color[1] + ratio * (max_color[1] - min_color[1]))
    blue = int(min_color[2] + ratio * (max_color[2] - min_color[2]))
    
    return red, green, blue

