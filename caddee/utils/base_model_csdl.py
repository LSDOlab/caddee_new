from csdl import Model
import csdl
from caddee.utils.helper_functions.camel_to_snake import camel_to_snake
from caddee.core.caddee_core.system_model.design_scenario.design_condition.design_condition import SteadyDesignCondition
from caddee.core.caddee_core.system_model.sizing_group.sizing_models.sizing_model import SizingModel
from caddee.core.caddee_core.system_model.design_scenario.design_condition.mechanics_group.mechanics_model.mechanics_model import MechanicsModel
# from caddee.core.caddee_core.system_model.design_scenario.design_condition.nonmechanics_group. import NonmechanicsGroup
from caddee.core.caddee_core.system_model.design_scenario.design_condition.power_group.power_model.power_model import PowerModel
import warnings
from lsdo_modules.module_csdl.module_csdl import ModuleCSDL
from caddee.core.caddee_core.system_representation.component.component import Component
from csdl import GraphRepresentation


class BaseModelCSDL(ModuleCSDL):
    def initialize(self): pass

    def define(self): pass

    def create_condition_csdl_input_variables(self, design_condition):
        """
        Method to create csdl input variables 
        
        Idea is that user-specified variables inside the run script are
        turned in csdl variables with csdl_model.create_input() at the 
        design_condition_csdl or system_representation level. 
        """
        
        # Looping over dictionary that contains csdl variable information
        variable_metadata_dictionary = design_condition.variables_metadata
        for csdl_var_name, csdl_var_val_dict in variable_metadata_dictionary.__dict__['_dict'].items():
            if csdl_var_val_dict['computed_upstream'] == True:
                pass
            else:
                csdl_var_val = csdl_var_val_dict['value']
                if isinstance(csdl_var_val, type(None)):
                    pass
                else:
                    promoted_name = design_condition.parameters['name'] + '_' + csdl_var_name
                    # if 'rpm' in csdl_var_name:
                    #     print(csdl_var_name)
                    self.create_input(promoted_name, csdl_var_val)
                    # Check if dv_flag is True
                    if csdl_var_val_dict['dv_flag'] == True:
                        lb = csdl_var_val_dict['lower']
                        ub = csdl_var_val_dict['upper']
                        scaler = csdl_var_val_dict['scaler']
                        self.add_design_variable(promoted_name, lb, ub, scaler)

    def create_component_csdl_input_variables(self, component_dictionary):
        """
        The goal of this method is to crate csdl variables that are 
        specific to a component but NOT a condition (e.g., radius)

        This method will most likely be depreciated in future versions 
        of CADDEE since component specific variables that aren't of 
        operational nature (like goemetric variables) will come from 
        upstream.
        """
        for comp_name, comp in component_dictionary.items():
            comp_variable_metadata_dictionary = comp.variables_metadata
            for csdl_var_name, csdl_var_val_dict in comp_variable_metadata_dictionary.__dict__['_dict'].items():
                # print('DEISNG CONDITION?-----', csdl_var_val_dict['design_condition'])
                # print('PROMOTED NAME-----',  comp_name + '_' + csdl_var_name)
                if csdl_var_val_dict['computed_upstream'] == True:
                    pass
                elif csdl_var_val_dict['design_condition']:
                    # print('Let us skip this one, it has a design condition')
                    pass
                else:
                    csdl_var_val = csdl_var_val_dict['value']
                    if isinstance(csdl_var_val, type(None)):
                        pass
                    else: 
                        promoted_name = comp_name + '_' + csdl_var_name
                        self.create_input(promoted_name, csdl_var_val)
                        # Check if dv_flag is True
                        if csdl_var_val_dict['dv_flag'] == True:
                            lb = csdl_var_val_dict['lower']
                            ub = csdl_var_val_dict['upper']
                            scaler = csdl_var_val_dict['scaler']
                            self.add_design_variable(promoted_name, lb, ub, scaler)

    def add_csdl_models(self, models_list, model_names_list, models_num_nodes_list=None):
        """
        Method to add csdl sub models which are vectorized.

        Arguments:
        --------
            - models_list : (list)
                list containing models (pure python class instance).
                Each model has a '_assemble_csdl()' method that returns
                the csdl model to be added.
                In addition, each model has a promoted_variables list to 
                promote certain variables that are not connected (like 
                geometric variables that are computed upstream, e.g., radius)
            - model_names_list : (list)
                list containing the names of the models 

        """
        num_models = len(models_list)
        for i in range(num_models):
            csdl_model_name = model_names_list[i]
            model = models_list[i]
            if models_num_nodes_list:
                model.num_nodes = models_num_nodes_list[0]
            csdl_model = model._assemble_csdl()
            promoted_vars = []
            for input, input_dict in model.inputs.input_variables.items():
                if input_dict['promoted_variable'] is True:
                    prefix = model.parameters['component'].parameters['name']
                    promoted_vars.append('{}_{}'.format(prefix, input))
            # if model.promoted_variables:
            #     prefix = model.parameters['component'].parameters['name']
            #     promoted_variables = [prefix + '_' + i for i in model.promoted_variables]
            #     promoted_vars = promoted_variables
            # else:
            #     promoted_vars = []
            
            self.add(csdl_model, csdl_model_name, promotes=promoted_vars)
        
    def add_csdl_sizing_models(self, models_dictionary):
        """
        Method to add sizing models (non-vectorized).
        
        On the csdl side, there is a different method for adding
        sizing models. This is because sizing models aren't vectorized.
        """
        for model_name, model in models_dictionary.items():
            promoted_vars = []
            for input, input_dict in model.inputs.input_variables.items():
                if input_dict['promoted_variable'] is True:
                    promoted_vars.append(input)
            # if model.promoted_variables:
            #     promoted_vars = model.promoted_variables
            # else:
            #     promoted_vars = []
            csdl_model = model._assemble_csdl()
            self.add(csdl_model, model_name, promotes=promoted_vars)

    # TODO: finalize computation of aircraft states from set of standarad inputs 
    def compute_aircraft_states(self, condition):
        variable_metadata_dictionary = condition.variables_metadata    
        name = condition.parameters['name']
        a = self.declare_variable(name + '_' + 'speed_of_sound', shape=(1, ))

        # print('SPEEED', variable_metadata_dictionary['speed'])
        # 3 ways to get cruise speed 
        #   1) If range and time are specified:
        if variable_metadata_dictionary['range'] and variable_metadata_dictionary['time']:
            # print('TESTING', variable_metadata_dictionary.__dict__['_dict']['range'])
            range = self.declare_variable(name + '_' + 'range', shape=(1, ))
            time = self.declare_variable(name + '_' + 'time', shape=(1, ))
            speed = range / time
        #   2) If Mach number is specified 
        elif variable_metadata_dictionary['mach_number'] is not None: # double check with OptionsDictionary
            mach_number = self.declare_variable(name + '_' + 'mach_number', shape=(1, ))
            speed = mach_number * a
        #   3) If cruise speed is specified
        if variable_metadata_dictionary['speed'] is not None:
            speed = self.declare_variable(name + '_' + 'speed', shape=(1, )) 
        
        if variable_metadata_dictionary.__dict__['_dict']['initial_altitude']:
            initial_altitude = self.declare_variable(name + '_' + 'initial_altitude', shape=(1, ))

        phi = self.declare_variable(name + '_' + 'roll_angle', shape=(1, ))
        theta = self.declare_variable(name + '_' + 'pitch_angle', shape=(1, ))
        psi = self.declare_variable(name + '_' + 'yaw_angle', shape=(1, ))
        gamma = self.declare_variable(name + '_' + 'flight_path_angle', shape=(1, ))
        psi_w = self.declare_variable(name + '_' + 'wind_angle', shape=(1, ))
        observer_location = self.declare_variable(name + '_' + 'observer_location', shape=(3, ))
        x = observer_location[0]
        y = observer_location[1]
        z = observer_location[2]

        alpha = theta - gamma 
        u = speed * csdl.cos(theta)
        self.register_output(name + '_u', u)
        v = speed * csdl.sin(psi - psi_w)
        self.register_output(name + '_v', v)
        w = speed * csdl.sin(theta)
        self.register_output(name + '_w', w)

        # TODO: generalize so that p, q, r aren't 0 by default
        self.register_output(name + '_p', u * 0)
        self.register_output(name + '_q', u * 0)
        self.register_output(name + '_r', u * 0)

        # NOTE: seems that a subset of the 12 aircraft states are actually caddee inputs 
        self.register_output(name + '_phi', phi * 1)
        self.register_output(name + '_gamma', gamma * 1)
        self.register_output(name + '_psi', psi * 1)
        self.register_output(name + '_theta', theta * 1)

        self.register_output(name + '_x', x * 1)
        self.register_output(name + '_y', y * 1)
        self.register_output(name + '_z', z * 1)
    
    def vectorize_inputs(self, group): 
        from caddee.core.caddee_core.system_model.design_scenario.design_condition.mechanics_group.mechanics_model.mechanics_model import MechanicsModel
        from caddee.core.caddee_core.system_model.design_scenario.design_condition.power_group.power_model.power_model import PowerModel
        model_names_list = group._all_models_names_list
        model_list = group._all_models_list 
        print('model_list', model_list)
        df = group._condition_solver_frame
        conditions_list = df.columns.tolist()
        num_nodes = df.shape[1]
        mech_model_counter = 0
        for i in range(len(model_list)):
            model = model_list[i]
            print('MODEL', model)
            print('MODEL INPUTS', model.inputs)
            print('Module', model.module)
            exit()
            model_name = model_names_list[i]
            selection_list = df.loc[[model_name]].values[0]
            # Check if the model is an instance of MechanicModel 
            if isinstance(model, MechanicsModel): 
                comp = model.parameters['component']
                comp_name = comp.parameters['name']
                # Vectorize aircraft states once for the first instance of a mechanics model
                if mech_model_counter == 0:
                    ac_states = ['u', 'v', 'w', 'p', 'q', 'r', 'phi', 'theta', 'psi', 'x', 'y', 'z']
                    for i in range(len(ac_states)):
                        csdl_var_vect = self.create_output(ac_states[i], shape=num_nodes)
                        for j in range(num_nodes):
                            name = '{}_{}'.format(conditions_list[j], ac_states[i])
                            csdl_var_vect[j] = self.declare_variable(name, shape=(1, ))
                    mech_model_counter += 1
                
                # Get any model specific variables 
                model_spec_inpts = []
                for input, input_dict in model.inputs.input_variables.items():
                    if input_dict['caddee_input'] is True:
                        model_spec_inpts.append(input)
                
                # Create the right name space: comp_name + '_' + var_name
                model_vect_inpts = [comp_name + '_' + i for i in model_spec_inpts]

                # Vectorize model specific variables 
                for i in range(len(model_vect_inpts)):
                    csdl_var_vect = self.create_output(model_vect_inpts[i], shape=num_nodes)
                    for j in range(num_nodes):
                        # Check if the model is used in the condition 
                        if selection_list[j] == 1:
                            name = '{}_{}'.format(conditions_list[j], model_vect_inpts[i])
                            csdl_var_vect[j] = self.declare_variable(name, shape=(1, ))
                        # If model is not used, assign value of 0 (need to declare variable with
                        # arbitrary but unique name)
                        else:
                            name = '{}_{}'.format(conditions_list[j], str(j))
                            csdl_var_vect[j] = self.declare_variable(name, val=0, shape=(1, ))
            
            # TODO: rotor and motor rpm: both should come from vectorization: Believe this is addressed
            # Else if check if the model is an instance of PowerModel 
            elif isinstance(model, PowerModel): 
                comp = model.parameters['component']
                comp_name = comp.parameters['name']
                
                # Get any model specific variables 
                model_spec_inpts = []
                for input, input_dict in model.inputs.input_variables.items():
                    if input_dict['caddee_input'] is True and input_dict['used_upstream'] is False:
                        model_spec_inpts.append(input)
                
                # Create the right name space: comp_name + '_' + var_name
                model_vect_inpts = [comp_name + '_' + i for i in model_spec_inpts]
                # print('POWER_MODEL_INPUTS', model_vect_inpts)
                # Vectorize model specific variables 
                for i in range(len(model_vect_inpts)):
                    csdl_var_vect = self.create_output(model_vect_inpts[i], shape=num_nodes)
                    for j in range(num_nodes):
                        # Check if the model is used in the condition 
                        if selection_list[j] == 1:
                            name = '{}_{}'.format(conditions_list[j], model_vect_inpts[i])
                            csdl_var_vect[j] = self.declare_variable(name, shape=(1, ))
                        # If model is not used, assign value of 0 (need to declare variable with
                        # arbitrary but unique name)
                        else:
                            name = '{}_{}'.format(conditions_list[j], str(j))
                            csdl_var_vect[j] = self.declare_variable(name, val=0, shape=(1, ))
            
        return


    def connect_vect_vars(self, group, connections=None):
        """
        Connect vectorized varialbes. 

        Only caddee_inputs are vectorized.
        """
        from caddee.core.caddee_core.system_model.design_scenario.design_condition.mechanics_group.mechanics_group import MechanicsGroup
        from caddee.core.caddee_core.system_model.design_scenario.design_condition.power_group.power_group import PowerGroup
        from builtins import any as b_any
        
        
        modules = group._all_models_list
        module_names = group._all_models_names_list

        if isinstance(group, MechanicsGroup):
            counter = 0
            for module in modules:
                if module.parameters.__contains__('component'):
                    comp = module.parameters['component']
                    if comp:
                        comp_name = comp.parameters['name']
                        comp_vars = comp.parameters['component_vars']
                else: #TODO: think about whether every model/modules needs a component
                    pass
                
                csdl_model = module._assemble_csdl()
                GraphRepresentation(csdl_model)
                vars = {**csdl_model.module_inputs, **csdl_model.module_declared_vars}
                for var, var_info in vars.items():
                    if var_info['vectorized'] is True and var not in csdl_model.promoted_vars:
                        if var in comp_vars:
                            self.connect(f'vectorization.{var}_{comp_name}', f'mechanics_group.{module_names[counter]}.{var}')
                        else:
                            self.connect(f'vectorization.{var}', f'mechanics_group.{module_names[counter]}.{var}')
                counter += 1

        if isinstance(group, PowerGroup):
            from_vars = []
            from_models = []
            to_models = []
            to_vars = []
            if connections:
                for connection in connections:
                    (_from, _from_var, _to, _to_var) = connection
                    if isinstance(_from, Component):
                        from_models.append(_from.parameters['name'])
                    if isinstance(_to, Component):
                        to_models.append(_to.parameters['name'])
                    to_vars.append(_to_var)
                    from_vars += _from_var
            
            counter = 0
            for module in modules:
                vect_prefix = None
                if module.parameters.__contains__('component'):
                    comp = module.parameters['component']
                    comp_name = comp.parameters['name']
                    comp_vars = comp.parameters['component_vars']
                    if comp_name in to_models:
                        vect_prefix = from_models[to_models.index(comp_name)]
                else:
                    pass
                
                csdl_model = module._assemble_csdl()
                GraphRepresentation(csdl_model)
                vars = {**csdl_model.module_inputs, **csdl_model.module_declared_vars}
                for var, var_info in vars.items():
                    if var_info['vectorized'] is True and var not in csdl_model.promoted_vars:
                        if var in comp_vars and vect_prefix is None:
                            self.connect(f'vectorization.{var}_{comp_name}', f'power_group.{module_names[counter]}.{var}')
                        elif var in comp_vars and vect_prefix:
                            self.connect(f'vectorization.{var}_{vect_prefix}', f'power_group.{module_names[counter]}.{var}')
                        elif var not in comp_vars and var in from_vars:
                            pass
                        else:
                            self.connect(f'vectorization.{var}', f'power_group.{module_names[counter]}.{var}')
                counter += 1
                # TODO: For improved efficiency: try to only call '_assemble_csdl()'
                # in one place. Right now, it's called at least 3 times to extract 
                # modules inputs, decalred variables etc...
        
        return

    def connect_model_vars(self,connections_list=None, sizing_group=None, mech_group=None, non_mech_group=None, pwr_group=None, name=None):
        """
        This method connects variables between models according to 
        the connections provided by the user in the run script.

        Arguments
        ----------
            mech_group : an instance of MechanicsGroup or None
            non_mech_group : an instance of NonMechanicsGroup or None
            pwr_group : an instance of PowerGroup or None
            connections_list : list containing model connections 

        Possible connections:
        -----------
            1) Power systems architecture nodal connections 
                Ex: rotor --> Motor 
            2) System-level connections system.connect()
                Ex: design condition --> sizing model 
            3) Sizing group level connections 
                Ex: battery sizing --> M4 regression 
            4) Mech-group connections 
                Ex: Aero <--> Struct (coupling)
        """

        # This happens for connecting non-vectorized variables e.g.,
        # from a design condition to (and between) sizing models 
        if not any([mech_group, non_mech_group, pwr_group]) and connections_list:
            # print('Connect system-level models')
            for connection in connections_list:
                upstream = connection[0]
                upstream_vars = connection[1]
                downstream = connection[2]
                downstream_vars = connection[3]
                
                # Upstream
                # If connecting from a design condition 
                if isinstance(upstream, SteadyDesignCondition):
                    des_cond_name = upstream.parameters['name']
                    for i in range(len(upstream_vars)):
                        upstream_var = upstream_vars[i] 
                        if upstream_var not in upstream.variables_metadata:
                            raise Exception('UNKNOWN CONNECTION!')
                        else:
                            if not name:
                                raise NotImplementedError
                            else:
                                upstream_connection_string = f"{name}.{des_cond_name}.{des_cond_name}_{upstream_var}"
                # If connecting from a sizing model 
                elif isinstance(upstream, SizingModel):
                    for i in range(len(upstream_vars)):
                        upstream_var = upstream_vars[i]
                        if upstream_var not in upstream.outputs.output_variables:
                            raise Exception('UNKNOWN CONNECTION!')
                        else:
                            upstream_connection_string = f"sizing_group.{camel_to_snake(type(upstream).__name__)}.{upstream_var}"


                # Downstream
                # If connecting to a design condition (not allowed)
                if isinstance(downstream, SteadyDesignCondition):
                    raise Exception('Cannot connect to a desing condition')
                # If connecting to a sizing model 
                elif isinstance(downstream, SizingModel):
                    for i in range(len(downstream_vars)):                
                        downstream_var = downstream_vars[i]
                        if downstream_var not in downstream.inputs.input_variables:
                            raise Exception((f"UNKNOWN CONNECTION TARGET {downstream_var}! Acceptable connection")
                                (f"targets are {list(downstream.inputs.input_variables.keys())}"))
                        else:
                            downstream_connection_string = f"sizing_group.{camel_to_snake(type(downstream).__name__)}.{downstream_var}"
                
                self.connect(upstream_connection_string, downstream_connection_string)
                # print(connection)
            

        elif not any([mech_group, non_mech_group, pwr_group, connections_list]) and sizing_group:
            for model_name, model in sizing_group.models_dictionary.items():
                self.connect(f"sizing_group.{model_name}.mass", f"mass_properties.{model_name}.mass")
                self.connect(f"sizing_group.{model_name}.cgx", f"mass_properties.{model_name}.cgx")
                self.connect(f"sizing_group.{model_name}.cgy", f"mass_properties.{model_name}.cgy")
                self.connect(f"sizing_group.{model_name}.cgz", f"mass_properties.{model_name}.cgz")
                self.connect(f"sizing_group.{model_name}.ixx", f"mass_properties.{model_name}.ixx")
                self.connect(f"sizing_group.{model_name}.iyy", f"mass_properties.{model_name}.iyy")
                self.connect(f"sizing_group.{model_name}.izz", f"mass_properties.{model_name}.izz")
                self.connect(f"sizing_group.{model_name}.ixz", f"mass_properties.{model_name}.ixz")

        else: 
            models = []
            model_names = []
            if mech_group:
                models += mech_group._all_models_list
                model_names += mech_group._all_models_names_list
            if non_mech_group:
                models += non_mech_group._all_models_list
                model_names += non_mech_group._all_models_names_list
            if pwr_group:
                models += pwr_group._all_models_list
                model_names += pwr_group._all_models_names_list
            
            # Setup: 
            # -> 'models' is a list of models across all groups
            # -> Each model has a component associated with it
            # -> 'connections_list' contains:
            #      - Upstream component or model 
            #      - Upstream string variable
            #      - Downstream component or model 
            #      - Downstream string variable
            # Algorithm
            #   for connection in connections_list:
            #       1) find upstream & downstream component in models list
            #       2) check if upstream and downstream string variables 
            #          are contained in 'inputs' & 'outputs' of the models
            #          ('inputs' & 'output' are variable groups)
            #       3) if connection string not in variables exist:
            #           raise Exception('Attempting to connect ...')
            #       4) create string name spaces
            #           
            
            for connection in connections_list:
                upstream_comp = connection[0]
                upstream_vars = connection[1]
                downstream_comp = connection[2]
                downstream_vars = connection[3]
                
                # Upstream model + name
                upstream_model = next(
                    model for model in models if model.parameters['component'] == upstream_comp
                )
                upstream_model_name = model_names[models.index(upstream_model)]
                
                # Downstream model + name
                downstream_model = next(
                    model for model in models if model.parameters['component'] == downstream_comp
                )
                downstream_model_name = model_names[models.index(downstream_model)]

                for i in range(len(upstream_vars)):
                    upstream_var = upstream_vars[i]
                    downstream_var = downstream_vars[i]
                    
                    # Check if user provided connection strings are valid (contained in model 'inputs'/'outputs')
                    if (upstream_var not in upstream_model.outputs.output_variables) and (upstream_var not in upstream_model.inputs.input_variables):
                        raise Exception(f"Unkown connection variable - Attempting to connect output variable '{upstream_var}' of model {upstream_model} " + 
                        f"but '{upstream_var}' is not a model input or output. This model has outputs: {list(upstream_model.outputs.output_variables.keys())} " +
                        f"and inputs :{list(upstream_model.inputs.inputs_variables.keys())}." + '\n' + "It is recommended to always connect outputs of the upstream " +
                        f"model to inputs of the downstream model. If the upstream variable is also an input, CADDEE assumes that this input is vectorized and will attempt " +
                        f"to connect the vectorized input.")
                    elif (upstream_var not in upstream_model.outputs.output_variables) and (upstream_var in upstream_model.inputs.input_variables):
                        upstream_connection_string = f"vectorization.{upstream_comp.parameters['name']}_{upstream_var}"
                        warnings.warn(f"Connecting variable '{upstream_var}' of model {upstream_model}. This variable is an input and not an output of the model. " +
                        f"CADDEE will attempt to connect the equivalent vectorized variable associated with the model/component.")
                    else:
                        # Create string names for connections 
                        if isinstance(upstream_model, MechanicsModel):
                            upstream_connection_string = f'mechanics_group.{upstream_model_name}.{upstream_var}'
                        elif isinstance(upstream_model, PowerModel):
                            upstream_connection_string = f'power_group.{upstream_model_name}.{upstream_var}'
                        else: 
                            raise NotImplementedError

                    if downstream_var not in downstream_model.inputs.input_variables:
                        raise Exception(f"Unkown connection variable - Attempting to connect to input variable '{downstream_var}' of model {downstream_model} " + 
                        f"but '{downstream_var}' is not a model input. This model has inputs: {list(downstream_model.inputs.input_variables.keys())}")
                    else:
                        if isinstance(downstream_model, MechanicsModel):
                            downstream_connection_string = f'mechanics_group.{downstream_model_name}.{downstream_var}'
                        elif isinstance(downstream_model, PowerModel):
                            downstream_connection_string = f'power_group.{downstream_model_name}.{downstream_var}'
                        else: 
                            raise NotImplementedError
                
                # Connect variabls
                    # print(upstream_connection_string)
                    # print(downstream_connection_string)
                    # print('--------------------')
                    self.connect(upstream_connection_string, downstream_connection_string)
                # print('upstream_model_type----', type(upstream_model))
                # Connect variables 
                # print('Upstream Model---------', upstream_model_name)
                # print('Downstream Model-------', downstream_model_name)
    
        pass

    def connect_csdl_vars(self, connect_from, connect_to):
        if len(connect_from) != len(connect_to):
            raise Exception("'connect_from' and 'connect_to' don't have the same number of elements")
        else:
            for i in range(len(connect_from)):
                self.connect(connect_from[i], connect_to[i])


    def _connect(self, connections, mech_group=None, non_mech_group=None, pwr_group=None):

        if self.parameters.__contains__('system_model'):
            system_model = self.parameters['system_model']
            design_scenario_dict = system_model.design_scenario_dictionary

        models = []
        model_names = []
        if mech_group:
            models += mech_group._all_models_list
            model_names += mech_group._all_models_names_list
        if non_mech_group:
            models += non_mech_group._all_models_list
            model_names += non_mech_group._all_models_names_list
        if pwr_group:
            models += pwr_group._all_models_list
            model_names += pwr_group._all_models_names_list

        for connection in connections:
            _from, _from_var, _to, _to_var = connection

            if len(_to_var) > 1 or len(_from_var) > 1:
                raise Exception(f'More than one connection at a time ({_to_var}, {_from_var}) not implemented yet.')
            
            # Case 1) Connecting non-vectorized variables 
            # Ex: Between sizing models or from a design condition
            # to a sizing model 
            if isinstance(_from, SteadyDesignCondition) and isinstance(_to, SizingModel):
                design_condition_name = _from.parameters['name']
                for design_scenario_name, design_scenario in design_scenario_dict.items():
                    if design_condition_name in design_scenario.design_condition_dictionary:
                        connect_from = f"{design_scenario_name}.{design_condition_name}.{design_condition_name}_{_from_var[0]}"
                        if _to.parameters.__contains__('component'):
                            comp = _to.parameters['component']
                            if comp is not None:
                                comp_name = comp.parameters['name']
                                mod_name = camel_to_snake(type(_to).__name__)
                                to_name = f'{comp_name}_{mod_name}'
                                connect_to = f"sizing_group.{to_name}.{_to_var[0]}"
                            else:
                                connect_to = f"sizing_group.{camel_to_snake(type(_to).__name__)}.{_to_var[0]}"
                        else:
                            raise NotImplementedError
                    else:
                        pass

                    self.connect(connect_from, connect_to)

            elif isinstance(_from, SizingModel) and isinstance(_to, SizingModel):
                connect_from = f"sizing_group.{camel_to_snake(type(_from).__name__)}.{_from_var[0]}"
                connect_to = f"sizing_group.{camel_to_snake(type(_to).__name__)}.{_to_var[0]}"
                
                self.connect(connect_from, connect_to)
            
            # Case 2) Powertrain architecture connections
            elif isinstance(_from, Component) and isinstance(_to, Component):
                # Upstream model + name
                upstream_model = next(
                    model for model in models if model.parameters['component'] == _from
                )
                upstream_model_name = model_names[models.index(upstream_model)]
                
                # Downstream model + name
                downstream_model = next(
                    model for model in models if model.parameters['component'] == _to
                )
                downstream_model_name = model_names[models.index(downstream_model)]

                for i in range(len(_from_var)):
                    upstream_var = _from_var[i]
                    downstream_var = _to_var[i]

                    # TODO: Implement error messages
                    # Create string names for connections 
                    if isinstance(upstream_model, MechanicsModel):
                        connect_from = f'mechanics_group.{upstream_model_name}.{upstream_var}'
                    elif isinstance(upstream_model, PowerModel):
                        connect_from = f'power_group.{upstream_model_name}.{upstream_var}'
                    else: 
                        raise NotImplementedError

                    if isinstance(downstream_model, MechanicsModel):
                        connect_to = f'mechanics_group.{downstream_model_name}.{downstream_var}'
                    elif isinstance(downstream_model, PowerModel):
                        connect_to = f'power_group.{downstream_model_name}.{downstream_var}'
                    else: 
                        raise NotImplementedError

                    self.connect(connect_from, connect_to)

            elif isinstance(_from, SizingModel) and isinstance(_to, Component):
                connect_from = connect_from = f"sizing_group.{camel_to_snake(type(_from).__name__)}.{_from_var[0]}"
                for design_scenario_name, design_scenario in design_scenario_dict.items():
                    mech_group, non_mech_group, pwr_group = design_scenario._assemble_groups(design_scenario.design_condition_dictionary)
                    models = []
                    model_names = []
                    if mech_group:
                        models += mech_group._all_models_list
                        model_names += mech_group._all_models_names_list
                    if non_mech_group:
                        models += non_mech_group._all_models_list
                        model_names += non_mech_group._all_models_names_list
                    if pwr_group:
                        models += pwr_group._all_models_list
                        model_names += pwr_group._all_models_names_list

                    if models:
                        downstream_model = next(
                            model for model in models if model.parameters['component'] == _to
                        )
                        downstream_model_name = model_names[models.index(downstream_model)]
                    else:
                        raise NotImplementedError
                    if isinstance(downstream_model, PowerModel):
                        connect_to = f'{design_scenario_name}.power_group.{downstream_model_name}.{_to_var[0]}'
                    else: 
                        raise Exception('Connecting from a sizing model to mechanics or non-mechanics model not yet implemented')
                
                self.connect(connect_from, connect_to)

            elif isinstance(_from, MechanicsModel) and isinstance(_to, MechanicsModel):
                from_model_comp_name = _from.parameters['component'].parameters['name']
                to_model_comp_name = _to.parameters['component'].parameters['name']
                connect_from = f"mechanics_group.{from_model_comp_name}_{camel_to_snake(type(_from).__name__)}.{_from_var[0]}"
                connect_to = f"mechanics_group.{to_model_comp_name}_{camel_to_snake(type(_to).__name__)}.{_to_var[0]}"
                
                print(connect_from)
                print(connect_to)
                self.connect(connect_from, connect_to)
            
            else:
                raise NotImplementedError
        # exit()

    def connect_sizing_to_mass_properties(self, sizing_group=None, mechanics_group=None):
        if sizing_group:
            for model_name, model in sizing_group.models_dictionary.items():
                self.connect(f"sizing_group.{model_name}.mass", f"constant_mass_properties.{model_name}.mass")
                self.connect(f"sizing_group.{model_name}.cgx", f"constant_mass_properties.{model_name}.cgx")
                self.connect(f"sizing_group.{model_name}.cgy", f"constant_mass_properties.{model_name}.cgy")
                self.connect(f"sizing_group.{model_name}.cgz", f"constant_mass_properties.{model_name}.cgz")
                self.connect(f"sizing_group.{model_name}.ixx", f"constant_mass_properties.{model_name}.ixx")
                self.connect(f"sizing_group.{model_name}.iyy", f"constant_mass_properties.{model_name}.iyy")
                self.connect(f"sizing_group.{model_name}.izz", f"constant_mass_properties.{model_name}.izz")
                self.connect(f"sizing_group.{model_name}.ixz", f"constant_mass_properties.{model_name}.ixz")
        elif mechanics_group:
            mech_models = mechanics_group._all_models_list
            mech_model_names = mechanics_group._all_models_names_list
            counter = 0
            for mech_model in mech_models:
                mech_model_name = mech_model_names[counter]
                if mech_model.parameters.__contains__('compute_mass_properties'):
                    if mech_model.parameters['compute_mass_properties'] is True:
                        self.connect(f"mechanics_group.{mech_model_name}.mass", f"varying_mass_properties.{mech_model_name}.mass")
                        self.connect(f"mechanics_group.{mech_model_name}.cgx", f"varying_mass_properties.{mech_model_name}.cgx")
                        self.connect(f"mechanics_group.{mech_model_name}.cgy", f"varying_mass_properties.{mech_model_name}.cgy")
                        self.connect(f"mechanics_group.{mech_model_name}.cgz", f"varying_mass_properties.{mech_model_name}.cgz")
                        self.connect(f"mechanics_group.{mech_model_name}.ixx", f"varying_mass_properties.{mech_model_name}.ixx")
                        self.connect(f"mechanics_group.{mech_model_name}.iyy", f"varying_mass_properties.{mech_model_name}.iyy")
                        self.connect(f"mechanics_group.{mech_model_name}.izz", f"varying_mass_properties.{mech_model_name}.izz")
                        self.connect(f"mechanics_group.{mech_model_name}.ixz", f"varying_mass_properties.{mech_model_name}.ixz")
                    else:
                        pass

                counter += 1
                
        else:
            raise NotImplementedError

    def connect_mass_properties_to_il_and_eom(self, design_scenario):
        name = design_scenario.parameters['name']
        self.connect(f"total_mass_properties.m_total", f"{name}.inertial_loads.m_total")
        self.connect(f"total_mass_properties.cgx_total", f"{name}.inertial_loads.cgx_total")
        self.connect(f"total_mass_properties.cgy_total", f"{name}.inertial_loads.cgy_total")
        self.connect(f"total_mass_properties.cgz_total", f"{name}.inertial_loads.cgz_total")

        if design_scenario.equations_of_motion_csdl:
            self.connect(f"total_mass_properties.ixx_total", f"{name}.eom.ixx_total")
            self.connect(f"total_mass_properties.ixz_total", f"{name}.eom.ixz_total")
            self.connect(f"total_mass_properties.iyy_total", f"{name}.eom.iyy_total")
            self.connect(f"total_mass_properties.izz_total", f"{name}.eom.izz_total")
            