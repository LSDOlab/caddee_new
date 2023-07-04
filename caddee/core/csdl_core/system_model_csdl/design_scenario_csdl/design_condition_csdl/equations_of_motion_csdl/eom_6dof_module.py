import m3l
import csdl
import caddee as cd
from caddee.core.csdl_core.system_model_csdl.design_scenario_csdl.design_condition_csdl.equations_of_motion_csdl.equations_of_motion_csdl import EulerFlatEarth6DoFGenRef
import copy


class EoMM3LEuler6DOF(m3l.ExplicitOperation):
    eom_model_name = 'euler_eom_gen_ref_pt'
    
    def initialize(self, kwargs):
        self.parameters.declare(name='name', default=self.eom_model_name)
        self.parameters.declare('num_nodes', types=int, default=1)

    def compute(self) -> csdl.Model:
        name = self.parameters['name']
        num_nodes = self.parameters['num_nodes']

        return EulerFlatEarth6DoFGenRef(name, num_nodes)

    def evaluate(self, total_mass, total_cg_vector, total_inertia_tensor, total_forces, total_moments, ac_states) -> m3l.Variable:
        self.name = self.eom_model_name
        mps_forces = {
            'total_mass' : total_mass,
            'total_cg_vector' : total_cg_vector,
            'total_inertia_tensor' : total_inertia_tensor,
            'total_forces' : total_forces,
            'total_moments' : total_moments,
        }

        ac_states_copy = copy.deepcopy(ac_states)
        del ac_states_copy['gamma']
        self.arguments = {**mps_forces, **ac_states_copy}


        
        # print(total_mass_1.name)
        # print(total_mass_2.name)

        trim_residual = m3l.Variable(name='trim_residual', shape=(1, ), operation=self)
        return trim_residual
        
    def compute_derivatives(self): pass
        

# NOTE: for later
class EoMM3LResidual(m3l.ImplicitOperation):
    def initialize(self): pass

    def evaluate_residual(self)-> csdl.Model:
        linmodel = ModuleCSDL()
        a_mat = linmodel.register_module_input('mp_matrix', shape=(6, 6))
        b_mat = linmodel.register_module_input('rhs', shape=(num_nodes, 6))
        state = linmodel.register_module_input('state', shape=(6, num_nodes))
        residual = csdl.matmat(a_mat, state) - csdl.transpose(b_mat)

        return linmodel
    
    def solve_residual_equations(self)-> csdl.Model: pass

    











if __name__ == "__main__":
    from lsdo_modules.module_csdl.module_csdl import ModuleCSDL
    pass