import m3l
import numpy as np
import csdl
import lsdo_geo as lg
from typing import List
from caddee.core.caddee_core.system_model.design_scenario.design_condition.design_condition import AcStates, AtmosphericProperties
from typing import Union, List
from dataclasses import dataclass


class DragComponent:
    def __init__(self, 
                 component_type : str, 
                 wetted_area : m3l.Variable,
                 characteristic_length : m3l.Variable, 
                 characteristic_diameter : Union[m3l.Variable, float, None] = None,
                 thickness_to_chord: Union[m3l.Variable, float, None] = None,
                 multiplicity :  int = 1,
                 Q : Union[int, float] = 1.1,
                 x_cm : Union[int, float] = 0.25,
                 sweep : Union[int, float, m3l.Variable] = 0.,
                 ) -> None:
        
        self.component_type = component_type
        self.wetted_area = wetted_area
        self.characteristic_length = characteristic_length
        self.characteristic_diameter = characteristic_diameter
        self.thickness_to_chord = thickness_to_chord
        self.multiplicity = multiplicity
        self.Q = Q
        self.x_cm = x_cm
        self.sweep = sweep

        acceptable_component_types = [
            'wing', 'tail', 'fuselage', 'boom', 'strut', 'pylon', 'flat_plate', 'canopy', 'nacelle'
        ]
        if component_type not in acceptable_component_types:
            raise ValueError(f"Unknown 'component_type' argument. Acceptable component types are {acceptable_component_types}")

        if (component_type in ['wing', 'tail', 'strut', 'pylon']) and (thickness_to_chord is None):
            raise ValueError(f"Component type '{component_type}' but did not specify 'thickness_to_chord'")

@dataclass
class DragBuildUpOutputs:
    D0 : m3l.Variable
    C_D0 : m3l.Variable
    forces : m3l.Variable
    moments : m3l.Variable = None


class DragBuildUpModel(m3l.ExplicitOperation):
    """
    Implementation of the Raymer drag-build up equations for computing an 
    estimate of the skin friction drag coefficient C_D0
    """
    def initialize(self, kwargs):
        self.parameters.declare('name', types=str)
        self.parameters.declare('num_nodes', types=int)
        self.parameters.declare('units', default='m', values=['ft', 'm'])

    def assign_attributes(self):
        self.name = self.parameters['name']
        self.num_nodes = self.parameters['num_nodes']
        self.units = self.parameters['units']

    def compute(self) -> csdl.Model:
        csdl_model = DragBuildUpCSDL(
            units=self.units,
            drag_comp_list=self.drag_comp_list,
            num_nodes=self.num_nodes
        )

        return csdl_model
    
    def evaluate(self, atmos: AtmosphericProperties, ac_states : AcStates, drag_comp_list : List[DragComponent], s_ref : m3l.Variable):
        self.arguments = {}
        self.arguments['density'] = atmos.density
        self.arguments['dynamic_viscosity'] = atmos.dynamic_viscosity
        self.arguments['a'] = atmos.speed_of_sound
        self.arguments['u'] = ac_states.u
        self.arguments['v'] = ac_states.v
        self.arguments['w'] = ac_states.w
        self.arguments['theta'] = ac_states.theta
        self.arguments['S_ref'] = s_ref

        for i, comp in enumerate(drag_comp_list):
            self.arguments[f"comp_{i}_s_wet"] = comp.wetted_area
            self.arguments[f"comp_{i}_l"] = comp.characteristic_length
            if isinstance(comp.characteristic_diameter, m3l.Variable):
                self.arguments[f"comp_{i}_d"] = comp.characteristic_diameter

            if isinstance(comp.thickness_to_chord, m3l.Variable):
                self.arguments[f"comp_{i}_t_c"] = comp.thickness_to_chord

        self.drag_comp_list = drag_comp_list


        D = m3l.Variable(name='D0', shape=(self.num_nodes, ), operation=self)
        C_D0 = m3l.Variable(name='C_D0', shape=(self.num_nodes, ), operation=self)
        F = m3l.Variable(name='F', shape=(self.num_nodes, 3), operation=self)
        # M = m3l.Variable(name='M', shape=(self.num_nodes, 3), operation=self)

        outputs = DragBuildUpOutputs(
            D0=D,
            C_D0=C_D0,
            forces=F,
        )

        return outputs

class DragBuildUpCSDL(csdl.Model):
    def initialize(self):
        self.parameters.declare('drag_comp_list', types=list)
        self.parameters.declare('num_nodes', types=int)
        self.parameters.declare('units', values=['m', 'ft'])
        self.parameters.declare('percent_laminar', types=float, default=0.)
        self.parameters.declare('percent_turbulent', types=float, default=1.)

    def define(self):
        percent_lam = self.parameters['percent_laminar']
        percent_turb = self.parameters['percent_turbulent']
        units = self.parameters['units']
        drag_comp_list = self.parameters['drag_comp_list']
        num_nodes = self.parameters['num_nodes']
        
        ft_2_to_m_2 = 0.092903
        ft_to_m = 0.3048

        if units == 'ft':
            S_ref = csdl.expand(self.declare_variable('S_ref', shape=(1, )), shape=(num_nodes, 1)) * ft_2_to_m_2
        else:
            S_ref = csdl.expand(self.declare_variable('S_ref', shape=(1, )), shape=(num_nodes, 1))
        
        
        rho = self.declare_variable('density', shape=(num_nodes, 1))
        mu = self.declare_variable('dynamic_viscosity', shape=(num_nodes, 1))
        a = self.declare_variable('a', shape=(num_nodes, 1))

        u = self.declare_variable('u', shape=(num_nodes, 1))
        v = self.declare_variable('w', shape=(num_nodes, 1))
        w = self.declare_variable('v', shape=(num_nodes, 1))

        V_inf = (u**2 + v**2 + w**2)**0.5
        q_inf = 0.5 * rho * V_inf**2
        M = V_inf / a

        theta = self.declare_variable('theta', shape=(num_nodes, 1))
        self.register_output('theta_test', theta * 1)

        C_D0 = self.create_input('C_D0_compute', val=0, shape=(num_nodes, 1))
        
        # Loop over components to compute C_D0
        for i, comp in enumerate(drag_comp_list):
            if units == 'ft':
                S_wet = csdl.expand(self.declare_variable(f"comp_{i}_s_wet", shape=(1, )), shape=(num_nodes, 1)) * ft_2_to_m_2
                l = csdl.expand(self.declare_variable(f"comp_{i}_l", shape=(1, )), shape=(num_nodes, 1)) * ft_to_m
            else:
                S_wet = csdl.expand(self.declare_variable(f"comp_{i}_s_wet", shape=(1, )), shape=(num_nodes, 1))
                l = csdl.expand(self.declare_variable(f"comp_{i}_l", shape=(1, )), shape=(num_nodes, 1))

            Q = comp.Q

            Re = rho * V_inf * l / mu

            if comp.component_type == 'flat_plate':
                Cf = 0.074 / Re**(1/5)

            elif comp.component_type in ['wing', 'strut', 'tail', 'pylon']:
                x_cm = comp.x_cm
                
                if isinstance(comp.sweep, m3l.Variable):
                    sweep = csdl.expand(self.declare_variable(f"comp_{i}_sweep", shape=(1, )), shape=(num_nodes, 1))
                elif isinstance(comp.sweep, float):
                    sweep = csdl.expand(self.create_input(f'comp_{i}_sweep', val=comp.sweep), shape=(num_nodes, 1))
                else:
                    raise TypeError(f"Wrong type for 'sweep'. Received type {type(comp.sweep)} but require types 'float' or 'm3l.Variable'")
                
                if isinstance(comp.thickness_to_chord, m3l.Variable):
                    t_c = csdl.expand(self.declare_variable(f"comp_{i}_t_c", shape=(1, )), shape=(num_nodes, 1))
                elif isinstance(comp.thickness_to_chord, float):
                    t_c = csdl.expand(self.create_input(f"comp_{i}_t_c",val=comp.thickness_to_chord), shape=(num_nodes, 1))
                else:
                    raise TypeError(f"Wrong type for 'thickness_to_chord'. Received type {type(comp.thickness_to_choed)} but require types 'float' or 'm3l.Variable'")
                
                
                Cf = 0.455 / (csdl.log10(Re)**2.58 * (1 + 0.144 * M**2)**0.65)
                FF = (1 + 0.6 / x_cm * t_c + 100 * t_c**4) * (1.34 * M**0.18 * csdl.cos(sweep)**0.28)
            
            else: 
                if isinstance(comp.characteristic_diameter, m3l.Variable):
                    if units == 'ft':
                        d = csdl.expand(self.declare_variable(f"comp_{i}_d", shape=(1, )), shape=(num_nodes, 1)) * ft_to_m
                    else:
                        d = csdl.expand(self.declare_variable(f"comp_{i}_d", shape=(1, )), shape=(num_nodes, 1))
                elif isinstance(comp.characteristic_diameter, float):
                    if units == 'ft':
                        d = comp.characteristic_diameter * ft_to_m
                    else:
                        d = comp.characteristic_diameter
                else:
                    raise NotImplementedError
                
                Cf = 0.455 / (csdl.log10(Re)**2.58 * (1 + 0.144 * M**2)**0.65)
                f = l/d

                if comp.component_type in ['fuselage', 'canopy', 'boom']:
                    FF = (1 - 60 / f**3 + f / 400)

                elif comp.component_type in ['nacelle']:
                    FF = 1 + 0.35/f
                else:
                    raise NotImplementedError
                
            self.register_output(f'Cf_{i}', Cf*1)
            self.register_output(f'FF_{i}', FF* 1)
            # self.print_var(Q)
            self.register_output(f'S_wet_{i}', S_wet* 1)
            C_D0 =  C_D0 + (Cf * FF *  Q *  S_wet) * comp.multiplicity

        C_D0 = C_D0 / S_ref
        D = C_D0 * S_ref * q_inf
        
        self.register_output('D0', D)
        self.register_output('C_D0', C_D0)

        # ref_pt = csdl.expand(self.declare_variable('ref_pt', shape=(3, ), val=np.array([0., 0., 0.])), shape=(num_nodes, 3), indices='i->ji')

        F = self.create_output('F', shape=(num_nodes, 3), val=0)
        for i in range(num_nodes):
            F[i, 0] = -D * csdl.cos(theta)
            F[i, 2] = D * csdl.sin(theta)



