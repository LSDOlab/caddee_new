import csdl
import m3l
from dataclasses import dataclass
import numpy as np


class StabilityAdapterModelCSDL(csdl.Model):
    def initialize(self):
        self.parameters.declare('arguments', types=dict)
        self.parameters.declare('num_nodes', types=int)
        self.parameters.declare('neglect', types=list, default=[])
    
    def define(self):
        args = self.parameters['arguments']
        num_nodes = self.parameters['num_nodes']
        ac_states = ['u', 'v', 'w', 'p', 'q', 'r', 'theta', 'phi', 'psi', 'x', 'y', 'z']
        special_cases = self.parameters['neglect']
        for key, value in args.items():
            if key in ac_states:
                csdl_var = self.declare_variable(key, shape=(num_nodes * 13, ))
                self.register_output(name=f'{key}_exp', var=csdl_var * 1)
            elif key in special_cases:
                csdl_var = self.declare_variable(key, shape=value.shape)
                self.register_output(name=f'{key}_exp', var=csdl_var * 1)
            else:
                csdl_var = self.declare_variable(key, shape=value.shape)
                if len(value.shape) == 1 and value.shape[0] == 1:
                    # print(key, value.shape)
                    csdl_var_exp = csdl.expand(csdl_var, shape=(num_nodes * 13, ))
                    self.register_output(name=f'{key}_exp', var=csdl_var_exp)
                elif len(value.shape) == 1 and value.shape[0] != 1:
                    # print(key, (13, ) + value.shape)
                    csdl_var_exp = csdl.reshape(csdl.expand(csdl_var, shape=(13, ) + value.shape, indices='i->ji'), new_shape=(13, value.shape[0]))
                    self.register_output(name=f'{key}_exp', var=csdl_var_exp)
                elif len(value.shape) == 2:
                    if num_nodes == value.shape[0]:
                        csdl_var_exp = csdl.reshape(csdl.expand(csdl_var, shape=(13, ) + value.shape, indices='ij->kij'), new_shape=(13*num_nodes, value.shape[1]))
                        self.register_output(name=f'{key}_exp', var=csdl_var_exp)
                    elif num_nodes == value.shape[1]:
                        csdl_var_exp = csdl.reshape(csdl.expand(csdl_var, shape=(13, ) + value.shape, indices='ij->kij'), new_shape=(13*num_nodes, value.shape[0]))
                        self.register_output(name=f'{key}_exp', var=csdl_var_exp)
                elif len(value.shape) > 2:
                    raise NotImplementedError

@dataclass
class C172AeroOutputs:
    forces : m3l.Variable = None
    moments : m3l.Variable = None

class C172AeroM3L(m3l.ExplicitOperation):
    def initialize(self, kwargs):
        # parameters
        self.parameters.declare('num_nodes', types=int, default=1)
        self._stability_flag = False
        super().initialize(kwargs=kwargs)

    def assign_attributes(self):
        self.name = self.parameters['name']

    def compute(self) -> csdl.Model:
        num_nodes = self.parameters['num_nodes']
        
        if self._stability_flag:
            csdl_model = StabilityAdapterModelCSDL(
                arguments=self.arguments,
                num_nodes=num_nodes,
            )

            solver_model =  C172AerodynamicsModelCSDL(
                num_nodes=num_nodes * 13,
                stability_flag=self._stability_flag,
            )

            operation_name = self.parameters['name']
            
            csdl_model.add(solver_model, operation_name, promotes=[])
            for key, value in self.arguments.items():
                csdl_model.connect(f'{key}_exp',f'{operation_name}.{key}')

        else:
            csdl_model =  C172AerodynamicsModelCSDL(
                num_nodes=num_nodes,
            )

        return csdl_model

    def evaluate(self, ac_states, delta_a, delta_r, delta_e) -> C172AeroOutputs:
        self.arguments = {}
        for key, value in ac_states.__dict__.items():
            if key in ['time', 'stability_flag']:
                pass
            else:
                self.arguments[key] = value
        self.arguments['delta_a'] = delta_a
        self.arguments['delta_e'] = delta_e
        self.arguments['delta_r'] = delta_r

        self._stability_flag = ac_states.stability_flag

        if self._stability_flag:
            num_nodes = self.parameters['num_nodes'] * 13
            forces = m3l.Variable(name=f'{self.name}.F', shape=(num_nodes, 3), operation=self)
            moments = m3l.Variable(name=f'{self.name}.M', shape=(num_nodes, 3), operation=self)

        else: 
            num_nodes = self.parameters['num_nodes']
            forces = m3l.Variable(name='F', shape=(num_nodes, 3), operation=self)
            moments = m3l.Variable(name='M', shape=(num_nodes, 3), operation=self)
        
        c172_aero_outputs = C172AeroOutputs(
            forces=forces,
            moments=moments
        )

        # if self._stability_flag:
        #     forces_perturbed = m3l.Variable(name='F_perturbed', shape=(8, 3), operation=self)
        #     moments_perturbed = m3l.Variable(name='M_perturbed', shape=(8, 3), operation=self)
        #     c172_aero_outputs.forces_perturbed = forces_perturbed
        #     c172_aero_outputs.moments_perturbed = moments_perturbed

        return c172_aero_outputs


class C172AerodynamicsModelCSDL(csdl.Model):

    def initialize(self):
        # self.parameters.declare(name='name', default='aerodynamics')
        self.parameters.declare('num_nodes', default=1)
        self.parameters.declare('stability_flag', types=bool, default=False)
        return
    def define(self):
        num_nodes = self.parameters['num_nodes']
        stability_flag = self.parameters['stability_flag']

        CL_q = 7.282
        Cm_q = -6.232
        CY_beta = -0.268
        CY_delta_rud = -0.561
        Cn_beta = 0.0126

        # Inputs constant across conditions (segments)
        Sw = self.create_input(name='wing_area', shape=(1,), val=16.2, units='m**2')
        chord = self.create_input(name='wing_chord', shape=(1,), val=1.49352, units='m')
        b = self.create_input(name='wing_span', shape=(1,), val=10.91184, units='m')

        # Inputs changing across conditions (segments)
        u = self.declare_variable(name='u',
                                       shape=(num_nodes, 1), units='rad', val=1)
        v = self.declare_variable(name='v',
                                  shape=(num_nodes, 1), units='rad', val=0)
        w = self.declare_variable(name='w',
                                  shape=(num_nodes, 1), units='rad', val=0)

        p = self.declare_variable(name='p',
                                  shape=(num_nodes, 1), units='rad', val=0)
        q = self.declare_variable(name='q',
                                  shape=(num_nodes, 1), units='rad', val=0)
        r = self.declare_variable(name='r',
                                  shape=(num_nodes, 1), units='rad', val=0)

        Phi = self.declare_variable(name='phi',
                                    shape=(num_nodes, 1), units='rad', val=0)
        Theta = self.declare_variable(name='theta',
                                      shape=(num_nodes, 1), units='rad', val=0)
        Psi = self.declare_variable(name='psi',
                                    shape=(num_nodes, 1), units='rad', val=0)

        gamma = self.declare_variable(name='gamma',
                                      shape=(num_nodes, 1), units='rad', val=0.)
        psiw = self.declare_variable(name='Psi_W',
                                     shape=(num_nodes, 1), units='rad', val=0)

        delta_e = self.declare_variable(name='delta_e',
                                        shape=(num_nodes, 1), units='rad', val=0)
        delta_r = self.declare_variable(name='delta_r',
                                        shape=(num_nodes, 1), units='rad', val=0)
        delta_a = self.declare_variable(name='delta_a',
                                        shape=(num_nodes, 1), units='rad', val=0)

        x = self.declare_variable(name='x',
                                  shape=(num_nodes, 1), units='rad', val=0)
        y = self.declare_variable(name='y',
                                  shape=(num_nodes, 1), units='rad', val=0)
        z = self.declare_variable(name='z',
                                  shape=(num_nodes, 1), units='rad', val=0)

        V = (u ** 2 + v ** 2 + w ** 2) ** 0.5

        self.register_output('dummy_1', psiw * Psi * Theta * gamma)
        # self.print_var(var=rho)
        rho = 1.225  # todo: compute as a function of altitude
        
        scaler_mat_numpy = np.ones((num_nodes, 1))
        scaler_mat_numpy[0, :] = 0
        scaler_mat = self.create_input('scaler_mat', val=scaler_mat_numpy)

        alpha = csdl.arctan(-w / u) # + scaler_mat * Theta # Theta - gamma #  Theta - gamma # 
        alpha_deg = alpha * 57.2958
        # self.print_var(Theta)
        self.register_output('alpha', alpha)
        # self.print_var(alpha_deg)
        beta =   csdl.arcsin(v / V) + scaler_mat * Psi # Psi + psiw #
        beta_deg = beta * 57.2958
        self.register_output('beta', beta)
        # self.print_var(alpha)
        # self.print_var(beta)
        delta_e_deg = delta_e * 57.2958
        delta_a_deg = delta_a * 57.2958
        delta_r_deg = delta_r * 57.2958

        wing_area = csdl.expand(var=Sw, shape=(num_nodes, 1))
        wing_chord = csdl.expand(var=chord, shape=(num_nodes, 1))
        wing_span = csdl.expand(var=b, shape=(num_nodes, 1))

        # Drag
        CD_alpha = 0.00033156 * alpha_deg ** 2 + \
                   0.00192141 * alpha_deg + \
                   0.03451242
        CD_delta_elev = 0.  # todo: fit a bivariate regression

        # Lift
        CL_alpha = 0.09460627 * alpha_deg + 0.16531678
        CL_delta_elev = -4.64968867e-06 * delta_e_deg ** 3 + \
                        3.95734084e-06 * delta_e_deg ** 2 + \
                        8.26663557e-03 * delta_e_deg + \
                        -1.81731015e-04

        # Pitching moment
        Cm_alpha = -0.00088295 * alpha_deg ** 2 + \
                   -0.01230759 * alpha_deg + \
                   0.01206867
        Cm_delta_elev = 1.11377133e-05 * delta_e_deg ** 3 + \
                        -9.96895700e-06 * delta_e_deg ** 2 + \
                        -2.03797109e-02 * delta_e_deg + \
                        1.37160466e-04

        # Side force
        CY_p = -0.00197933 * alpha_deg - \
               0.04682025
        CY_r = -3.30190866e-05 * alpha_deg ** 2 + \
               1.04792022e-03 * alpha_deg + \
               2.11499674e-01

        # Rolling moment
        Cl_beta = 3.64357755e-06 * alpha_deg ** 3 - \
                  3.62685593e-05 * alpha_deg ** 2 - \
                  3.54261202e-03 * alpha_deg \
                  - 2.01784324e-01
        Cl_p = 9.16035857e-05 * alpha_deg ** 3 - \
               3.41883453e-04 * alpha_deg ** 2 - \
               5.84495802e-03 * alpha_deg - \
               4.74431977e-01
        Cl_r = -1.82024434e-05 * alpha_deg ** 3 + \
               1.81520953e-04 * alpha_deg ** 2 + \
               1.84979559e-02 * alpha_deg + \
               2.88963441e-02
        Cl_delta_rud = 0.00394572 * alpha_deg - 0.06239875
        Cl_delta_aile = 0.00458196 * delta_a_deg - 0.00890937

        # Yawing moment
        Cn_p = 6.63631918e-08 * alpha_deg ** 3 \
               - 7.31768656e-05 * alpha_deg ** 2 \
               - 5.61479696e-03 * alpha_deg \
               - 9.48564775e-03
        Cn_r = 2.66064135e-07 * alpha_deg ** 4 \
               - 2.92479486e-06 * alpha_deg ** 3 \
               - 1.06671518e-04 * alpha_deg ** 2 \
               - 4.56040085e-04 * alpha_deg \
               - 2.74022830e-02
        Cn_delta_rud = -3.41409637e-05 * alpha_deg ** 2 \
                       + 1.11346991e-03 * alpha_deg \
                       + 2.21121678e-01
        Cn_delta_aile = 0.  # todo: fit a bivariate regression

        # Final aerodynamic coefficients
        CL = CL_alpha + CL_delta_elev + wing_chord / (2 * V+1e-4) * (CL_q * q)
        CD = CD_alpha + CD_delta_elev
        Cm = Cm_alpha + Cm_delta_elev + wing_chord / (2 * V+1e-4) * (2 * Cm_q * q)

        CY = (
                CY_beta * beta_deg +
                CY_delta_rud * delta_r_deg +
                wing_span / (2 * V + 1e-4) * (CY_p * p + CY_r * r)
        )
        Cl = (
                0.1 * Cl_beta * beta_deg +
                Cl_delta_aile * delta_a_deg +
                0.075 * Cl_delta_rud * delta_r_deg +
                wing_span / (2 * V+1e-4) * (Cl_p * p + Cl_r * r)
        )
        self.register_output('Cl', Cl)
        self.register_output('Cl_delta_aile', Cl_delta_aile)
        self.register_output('Cl_delta_rud', Cl_delta_rud)
        self.register_output('delta_r_deg', delta_r_deg)

        Cn = (
                Cn_beta * beta_deg +
                Cn_delta_aile +
                0.075 * Cn_delta_rud * delta_r_deg +
                wing_span / (2 * (V+1e-4)) * (Cn_p * p + Cn_r * r)
        )

        qBar = 0.5 * rho * V ** 2
        L = qBar * wing_area * CL \
            + x * 0. + y * 0. + z * 0. + Phi * 0. # Need to use these variables "somewhere" or CSDL throws an error
        D = qBar * wing_area * CD
        Y = qBar * wing_area * CY
        l = qBar * wing_area * wing_span * Cl
        m = qBar * wing_area * wing_chord * Cm
        n = qBar * wing_area * wing_span * Cn

        F_wind = self.create_output(name='F_wind', shape=(num_nodes, 3))
        F_wind[:, 0] = -D   
        F_wind[:, 1] = Y 
        F_wind[:, 2] = -L 

        M_wind = self.create_output(name='M_wind', shape=(num_nodes, 3))
        M_wind[:, 0] = l 
        M_wind[:, 1] = m 
        M_wind[:, 2] = n 

        F = self.create_output(name='F_compute', shape=(num_nodes, 3))
        M = self.create_output(name='M_compute', shape=(num_nodes, 3))

        # alpha = Theta - gamma
        # beta = Psi + psiw

        for ii in range(num_nodes):
            # https://www.mathworks.com/help/aeroblks/directioncosinematrixbodytowind.html
            DCM_bw = self.create_output(name=f'DCM_body_to_wind_{ii}', shape=(3, 3), val=0)
            DCM_bw[0:1, 0:1] = csdl.cos(alpha[ii, 0]) * csdl.cos(beta[ii, 0])
            DCM_bw[0:1, 1:2] = csdl.sin(beta[ii, 0])
            DCM_bw[0:1, 2:3] = csdl.sin(alpha[ii, 0]) * csdl.cos(beta[ii, 0])
            DCM_bw[1:2, 0:1] = -csdl.cos(alpha[ii, 0]) * csdl.sin(beta[ii, 0])
            DCM_bw[1:2, 1:2] = csdl.cos(beta[ii, 0])
            DCM_bw[1:2, 2:3] = -csdl.sin(alpha[ii, 0]) * csdl.sin(beta[ii, 0])
            DCM_bw[2:3, 0:1] = -csdl.sin(alpha[ii, 0])
            DCM_bw[2:3, 1:2] = alpha[ii, 0] * 0
            DCM_bw[2:3, 2:3] = csdl.cos(alpha[ii, 0])
            F[ii, :] = csdl.reshape(csdl.matvec(csdl.transpose(DCM_bw), csdl.reshape(F_wind[ii, :], (3,))), (1, 3))
            M[ii, :] = csdl.reshape(csdl.matvec(csdl.transpose(DCM_bw), csdl.reshape(M_wind[ii, :], (3,))), (1, 3))
    
        self.register_output(name='F', var=F*1)
        self.register_output(name='M', var=M*1)



if __name__ == "__main__":
    from python_csdl_backend import Simulator
    c172_aero_model = C172AerodynamicsModel()
    csdl_model = c172_aero_model._assemble_csdl()
    sim = Simulator(csdl_model, analytics=True, display_scripts=True)
    sim['u'] = 50.
    sim.run()
    print(sim['F'])