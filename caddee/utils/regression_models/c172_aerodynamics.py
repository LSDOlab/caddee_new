from lsdo_modules.module_csdl.module_csdl import ModuleCSDL
import csdl
import m3l


class C172AeroM3L(m3l.ExplicitOperation):
    def initialize(self, kwargs):
        # parameters
        self.parameters.declare('component', default=None, types=None)
        self.num_nodes = 1

    def compute(self) -> csdl.Model:
        return C172AerodynamicsModelCSDL(
            module=self,
        )

    def evaluate(self, ac_states):
        operation_csdl = self.compute()
        arguments = ac_states

        c172_aero_operation = m3l.CSDLOperation(name='c172_aero_model', arguments=arguments, operation_csdl=operation_csdl)
        forces = m3l.Variable(name='F', shape=(self.num_nodes, 3), operation=c172_aero_operation)
        moments = m3l.Variable(name='M', shape=(self.num_nodes, 3), operation=c172_aero_operation)

        return forces, moments


class C172AerodynamicsModelCSDL(ModuleCSDL):

    def initialize(self):
        self.parameters.declare(name='name', default='aerodynamics')
        self.parameters.declare('num_nodes', default=1)
        return
    def define(self):
        num_nodes = self.parameters['num_nodes']

        CL_q = 7.282
        Cm_q = -6.232
        CY_beta = -0.268
        CY_delta_rud = -0.561
        Cn_beta = 0.0126

        # Inputs constant across conditions (segments)
        Sw = self.declare_variable(name='wing_area', shape=(1,), val=16.2, units='m**2')
        chord = self.declare_variable(name='wing_chord', shape=(1,), val=1.49352, units='m')
        b = self.declare_variable(name='wing_span', shape=(1,), val=10.91184, units='m')

        # Inputs changing across conditions (segments)
        u = self.register_module_input(name='u',
                                       shape=(num_nodes, 1), units='rad', val=50, promotes=True)
        v = self.register_module_input(name='v',
                                  shape=(num_nodes, 1), units='rad', val=0)
        w = self.register_module_input(name='w',
                                  shape=(num_nodes, 1), units='rad', val=0)

        p = self.register_module_input(name='p',
                                  shape=(num_nodes, 1), units='rad', val=0)
        q = self.register_module_input(name='q',
                                  shape=(num_nodes, 1), units='rad', val=0)
        r = self.register_module_input(name='r',
                                  shape=(num_nodes, 1), units='rad', val=0)

        Phi = self.register_module_input(name='phi',
                                    shape=(num_nodes, 1), units='rad', val=0)
        Theta = self.register_module_input(name='theta',
                                      shape=(num_nodes, 1), units='rad', val=0)
        Psi = self.register_module_input(name='psi',
                                    shape=(num_nodes, 1), units='rad', val=0)

        gamma = self.register_module_input(name='gamma',
                                      shape=(num_nodes, 1), units='rad', val=0.)
        psiw = self.declare_variable(name='Psi_W',
                                     shape=(num_nodes, 1), units='rad', val=0)

        delta_e = self.register_module_input(name='delta_e',
                                        shape=(num_nodes, 1), units='rad', val=0, computed_upstream=False)
        delta_r = self.register_module_input(name='delta_r',
                                        shape=(num_nodes, 1), units='rad', val=0, computed_upstream=False)
        delta_a = self.register_module_input(name='delta_a',
                                        shape=(num_nodes, 1), units='rad', val=0, computed_upstream=False)

        x = self.register_module_input(name='x',
                                  shape=(num_nodes, 1), units='rad', val=0)
        y = self.register_module_input(name='y',
                                  shape=(num_nodes, 1), units='rad', val=0)
        z = self.register_module_input(name='z',
                                  shape=(num_nodes, 1), units='rad', val=0)

        V = (u ** 2 + v ** 2 + w ** 2) ** 0.5


        # self.print_var(var=rho)
        rho = 1.225

        alpha = Theta - gamma
        alpha_deg = alpha * 57.2958

        beta = Psi + psiw
        beta_deg = beta * 57.2958

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
        CL = CL_alpha + CL_delta_elev + wing_chord / (2 * V) * (CL_q * q)
        CD = CD_alpha + CD_delta_elev
        Cm = Cm_alpha + Cm_delta_elev + wing_chord / (2 * V) * (2 * Cm_q * q)

        CY = (
                CY_beta * beta_deg +
                CY_delta_rud * delta_r_deg +
                wing_span / (2 * V) * (CY_p * p + CY_r * r)
        )
        Cl = (
                0.1 * Cl_beta * beta_deg +
                Cl_delta_aile +
                0.075 * Cl_delta_rud * delta_r_deg +
                wing_span / (2 * V) * (Cl_p * p + Cl_r * r)
        )
        self.register_module_output('Cl', Cl)
        self.register_module_output('Cl_delta_aile', Cl_delta_aile)
        self.register_module_output('Cl_delta_rud', Cl_delta_rud)
        self.register_module_output('delta_r_deg', delta_r_deg)

        Cn = (
                Cn_beta * beta_deg +
                Cn_delta_aile +
                0.075 * Cn_delta_rud * delta_r_deg +
                wing_span / (2 * V) * (Cn_p * p + Cn_r * r)
        )

        qBar = 0.5 * rho * V ** 2
        L = qBar * wing_area * CL + x * 0.
        D = qBar * wing_area * CD + y * 0.
        Y = qBar * wing_area * CY + z * 0.
        l = qBar * wing_area * wing_span * Cl + Phi * 0.
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

        F = self.register_module_output(name='F', shape=(num_nodes, 3))
        M = self.register_module_output(name='M', shape=(num_nodes, 3))

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
        
        # F[:, 1] = Y * 0
        
        # M[:, 0] = Y * 0
        # M[:, 1] = Y * 0
        # M[:, 2] = Y * 0
        
        return


if __name__ == "__main__":
    from python_csdl_backend import Simulator
    c172_aero_model = C172AerodynamicsModel()
    csdl_model = c172_aero_model._assemble_csdl()
    sim = Simulator(csdl_model, analytics=True, display_scripts=True)
    sim['u'] = 50.
    sim.run()
    print(sim['F'])