import caddee.api as cd
from typing import Union


def make_ffd_block(
        components: Union[list, cd.Component], 
        num_control_points: tuple, 
        order: tuple, 
        xyy_to_uvw: tuple, 
        enclosure_volume_type: str = 'cartesian', 
        ffd_block_type: str = 'SRBG'
    ) -> cd.SRBGFFDBlock:
    """
    Helper function to create a B-spline FFD (Free-Form Deformation) block.

    Parameters
    ----------
    components : list or CADDEE component
        List of components to be used in the FFD block or single component.

    num_control_points : tuple of length three
        Specifies the number of control points in each direction (u, v, w).

    order : tuple of three integers
        Specifies the order of the control points in each direction.
        (Must be less than or equal to num_control_points.)

    xyz_to_uvw : tuple of three integers
        Specifies which axes in physical space correspond to which axes in parametric space.
        (Note: The first entry always corresponds to the 'long' axis.)
        Example: If the fuselage of an aircraft is oriented along the x-axis,
        then xyz_to_uvw = (0, 1, 2) or xyz_to_uvw = (0, 2, 1) would be acceptable.

    enclosure_volume_type : str, optional (default: 'cartesian')
        Specifies the type of enclosure volume.
        Only cartesian enclosure volumes are supported in this version.

    ffd_block_type : str, optional (default: 'SRBG')
        Specifies the type of FFD block.
        Only SRBG (Structured Rectangular B-Spline Grid) FFD blocks are supported in this version.
    
    Returns
    ----------
        FFD block (SRBG by default)
    """


    # Error checking 
    if not isinstance(components, (list, cd.Component)):
        raise TypeError(f"Argument 'components' needs to be a list of CADDEE components or asingle CADDEE component. Received type '{type(components)}'")
    elif isinstance(components, list):
        if not all(isinstance(item, cd.Component) for item in components):
            raise TypeError(f"All elements of the 'components' list argument need to be of type {cd.Component}")

    if not isinstance(enclosure_volume_type, str):
        raise TypeError(f"Argument 'enclosure_volume' needs to be of type '{str}', received type '{type(enclosure_volume_type)}'")
    elif enclosure_volume_type != 'cartesian':
        raise Exception(f"Only 'cartesian' enclosure volumes are supported, specified {enclosure_volume_type}")

    # TODO: Finish error checking


    # Creating the ffd_block
    if isinstance(components, list):
        primitives_dict = {}
        ffd_block_name = ''
        for component in components:
            name = component.parameters['name']
            primitives = component.get_geometry_primitives()
            primitives_dict.update(primitives)
            ffd_block_name += f'_{name}'
        
        ffd_block_name += '_ffd_block'
    else:
        primitives_dict = components.get_geometry_primitives
        ffd_block_name = f"{components.parameters['name']}_ffd_block"

    ffd_bspline_volume = cd.create_cartesian_enclosure_volume(
        enclosed_entities=primitives_dict, 
        num_control_points=num_control_points, 
        order=order, 
        xyz_to_uvw_indices=xyy_to_uvw
    )
    
    
    ffd_block = cd.SRBGFFDBlock(name=ffd_block_name, primitive=ffd_bspline_volume, embedded_entities=primitives_dict)

    return ffd_block


if __name__ == '__main__':
    import caddee.api as cd
    from caddee import IMPORTS_FILES_FOLDER

    lpc_rep = cd.SystemRepresentation()
    lpc_param = cd.SystemParameterization(system_representation=lpc_rep)

    file_name = IMPORTS_FILES_FOLDER / 'LPC_final_custom_blades.stp'
    spatial_rep = lpc_rep.spatial_representation
    spatial_rep.import_file(file_name=file_name)
    spatial_rep.refit_geometry(file_name=file_name)

    lpc_rep.make_components(
        fuselage='Fuselage_***.main',
        wing='Wing',
        h_tail='Tail_1',
    )

    wing = lpc_rep.components.wing
    
