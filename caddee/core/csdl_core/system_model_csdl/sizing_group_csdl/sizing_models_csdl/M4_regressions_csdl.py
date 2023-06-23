from csdl import Model
import csdl
from lsdo_modules.module_csdl.module_csdl import ModuleCSDL


class M4RegressionsCSDL(ModuleCSDL):
    def initialize(self):
        self.parameters.declare('name', default='M4_sizing', types=str)

    def define(self):
        shape = (1,)
        
        # Unit conversions
        ft_2_to_m_2 = 1 / 10.764
        ft_to_m = 0.3048
        ktas_to_m_s = 1 / 1.944
        lbs_to_kg =  1 / 2.205

        wing_area = self.register_module_input('wing_area', shape=shape, units='m^2', val=210 * ft_2_to_m_2) 
        # self.print_var(wing_area)
        wing_AR = self.register_module_input('wing_AR', shape=shape, val=13)
        # self.print_var(wing_AR)
        
        # Declaring input variables
        fuselage_length = self.register_module_input('fuselage_length', shape=shape, units='m' ,val=30 * ft_to_m)
        # fuselage_length = self.create_input('fuselage_length', shape=shape, units='m' ,val=30 * ft_to_m) 
        battery_mass = self.register_module_input('battery_mass', shape=shape, units='kg')
        # self.print_var(battery_mass)
        cruise_speed = self.register_module_input('cruise_speed', shape=shape, units='m/s', val=112 * ktas_to_m_s)
        # self.print_var(cruise_speed)
        tail_area = self.register_module_input('tail_area', shape=shape, units='m^2', val=39.51 * ft_2_to_m_2) * ft_2_to_m_2
        fin_area = self.register_module_input('fin_area',shape=shape, units='m^2', val=27.34 * ft_2_to_m_2)
        

        # Mass properties regression coefficients for: W_structural = f(A_wing, AR, l_fuselage, V_inf, W_battery)
        wing_boom_fuselage_structural_dict = {
            'wing_boom_fuselage_structural_mass': [1.67007499e+01 , 3.83791153e+01 , 3.92977635e+01,  3.34124324e-02, -3.39500691e-01, -450.72406965615073],
            'wing_boom_fuselage_structural_cg_x': [ 8.71049173e-03, -2.36099825e-02 , 3.50331188e-01, -1.49404879e-05, 6.33507205e-04, 0.5481315781098437],
            'wing_boom_fuselage_structural_cg_z': [7.14698638e-03 , 1.86416938e-02 , 2.60344332e-02, -2.74597877e-05, 3.70371492e-04, 1.5231177258191633],
            'wing_boom_fuselage_structural_I_xx_global': [4.64128802e+02 , 7.48816768e+02 , 4.27406278e+01 , 1.95057674e-01, -3.22651188e+00, -12566.768649954247],
            'wing_boom_fuselage_structural_I_yy_global': [1.96141041e+01 , 6.08270812e+01 , 2.97969925e+02,  2.23808375e-01, -1.17877102e-02,-2373.590644715917],
            'wing_boom_fuselage_structural_I_zz_global': [4.75197995e+02 , 7.87050706e+02 , 2.88472536e+02,  4.23056719e-01, -2.76568843e+00, -14640.638044226176],
        }

        # Mass properties regression coefficients for: W_nsm = f(A_wing, AR, l_fuselage, V_inf, W_battery)
        nsm_dict = {
            'nsm_cg_x': [5.40697511e-08, -9.31460124e-09 ,2.65320354e-01, 3.31403380e-09, 3.92993203e-08,0.4196136998833664],
            'nsm_cg_z': [-6.26057448e-08 , 2.25763192e-08 , 3.94301344e-02, -3.09382853e-09, -3.81768906e-08, 0.937567264623459],
            'nsm_I_xx_global': [6.73956224 ,15.1205628,  23.86830321 , 0.07380454 , 0.70349213, 16.749397163510935],
            'nsm_I_yy_global': [1.20985744e+01, -3.24862392e+01,  9.86825867e+02 , 5.42442194e-01, 1.54581997e+01, -5025.14817129927],
            'nsm_I_zz_global': [ 5.35901270e+00 ,-4.76067972e+01 , 9.62957564e+02 , 4.68637642e-01, 1.47547076e+01, -5041.89761780508],
            'nsm_I_xz_global': [7.6226027, 1.67336473 ,162.13675581 , 0.18919279 ,  4.46030064, -1032.7075717782232]
        }
        

        # Mass properties regression coefficients for W_empennage = f(A_vtail,A_htail)
        empennage_dict = {
            'empennage_struct_mass': [8.43266623, 10.05410839, -0.19944806479469435],
            'empennage_struct_cg_x': [0.06009468, 0.16721273, 8.076385644266017],
            'empennage_struct_cg_z': [-0.04459321,0.11557108, 2.6036382558118643],
            'empennage_struct_I_xx_global': [21.06581121, 26.25757046, -57.784444239569666],
            'empennage_struct_I_yy_global': [211.93126987, 340.19327823,-312.77053688616934],
            'empennage_struct_I_zz_global': [229.57588907, 313.00912756, -324.3336575937785],
            'empennage_struct_I_xz_global': [ 9.80677997,86.35964989,-68.79114643137058],
        }

        # Looping over dictionaries to compute mass properties and registering them as outputs
        for str, reg_coeff in wing_boom_fuselage_structural_dict.items():
            pred = reg_coeff[0] * wing_area + reg_coeff[1] * wing_AR + reg_coeff[2] * fuselage_length + reg_coeff[3] * battery_mass + reg_coeff[4] * cruise_speed + reg_coeff[5]
            self.register_module_output(str, pred)

        for str, reg_coeff in empennage_dict.items():
            pred = reg_coeff[0] * tail_area + reg_coeff[1] * fin_area + reg_coeff[2] 
            self.register_module_output(str, pred)

        for str, reg_coeff in nsm_dict.items():
            pred = reg_coeff[0] * wing_area + reg_coeff[1] * wing_AR + reg_coeff[2] * fuselage_length + reg_coeff[3] * battery_mass + reg_coeff[4] * cruise_speed + reg_coeff[5]
            self.register_module_output(str, pred)
               
        # Wing, booms, fuselage structural
        wing_boom_fuselage_structural_mass = self.register_module_input('wing_boom_fuselage_structural_mass', shape=shape)
        wing_boom_fuselage_structural_cg_x = self.register_module_input('wing_boom_fuselage_structural_cg_x', shape=shape)
        wing_boom_fuselage_structural_cg_y = self.register_module_input('wing_boom_fuselage_structural_cg_y', shape=shape, val=0.0)
        wing_boom_fuselage_structural_cg_z = self.register_module_input('wing_boom_fuselage_structural_cg_z', shape=shape,)
        wing_boom_fuselage_structural_I_xx_global = self.register_module_input('wing_boom_fuselage_structural_I_xx_global', shape=shape)
        wing_boom_fuselage_structural_I_yy_global = self.register_module_input('wing_boom_fuselage_structural_I_yy_global', shape=shape)
        wing_boom_fuselage_structural_I_zz_global = self.register_module_input('wing_boom_fuselage_structural_I_zz_global', shape=shape)
        wing_boom_fuselage_structural_I_xy_global = self.register_module_input('wing_boom_fuselage_structural_I_xy_global', shape=shape, val=0.043383253255396936)
        wing_boom_fuselage_structural_I_xz_global = self.register_module_input('wing_boom_fuselage_structural_I_xz_global', shape=shape, val=71.2668874236245)
        wing_boom_fuselage_structural_I_yz_global = self.register_module_input('wing_boom_fuselage_structural_I_yz_global', shape=shape, val=-0.07399825214427834)

        # Empennage Structural 
        empennage_mass = self.register_module_input('empennage_struct_mass', shape=shape)
        empennage_struct_cg_x = self.register_module_input('empennage_struct_cg_x',shape=shape)
        empennage_struct_cg_y = self.register_module_input('empennage_struct_cg_y',shape=shape,val=0.0)
        empennage_struct_cg_z = self.register_module_input('empennage_struct_cg_z',shape=shape)
        empennage_struct_I_xx_global = self.register_module_input('empennage_struct_I_xx_global',shape=shape)
        empennage_struct_I_yy_global = self.register_module_input('empennage_struct_I_yy_global',shape=shape)
        empennage_struct_I_zz_global = self.register_module_input('empennage_struct_I_zz_global',shape=shape)
        empennage_struct_I_xy_global = self.register_module_input('empennage_struct_I_xy_global',shape=shape,val=0.004868287408908892)
        empennage_struct_I_xz_global = self.register_module_input('empennage_struct_I_xz_global',shape=shape)
        empennage_struct_I_yz_global = self.register_module_input('empennage_struct_I_yz_global',shape=shape,val=0.0034687070026175955)

        # NSM
        nsm_mass = self.register_module_input('nsm_mass', shape=shape, val=1339.607080322429)
        nsm_cg_x = self.register_module_input('nsm_cg_x', shape=shape)
        nsm_cg_y = self.register_module_input('nsm_cg_y', shape=shape, val=0.0)
        nsm_cg_z = self.register_module_input('nsm_cg_z', shape=shape,)
        nsm_I_xx_global = self.register_module_input('nsm_I_xx_global', shape=shape)
        nsm_I_yy_global = self.register_module_input('nsm_I_yy_global', shape=shape)
        nsm_I_zz_global = self.register_module_input('nsm_I_zz_global', shape=shape)
        nsm_I_xy_global = self.register_module_input('nsm_I_xy_global', shape=shape, val=-0.0715791329553327)
        nsm_I_xz_global = self.register_module_input('nsm_I_xz_global', shape=shape)
        nsm_I_yz_global = self.register_module_input('nsm_I_yz_global', shape=shape, val=-0.038325317735083846)

        # Combining outputs from empennage, wing_boom_fuselage and nsm regressions 
        total_mass = (wing_boom_fuselage_structural_mass + empennage_mass) * 1.22 + nsm_mass
        cg_x = ((wing_boom_fuselage_structural_cg_x * wing_boom_fuselage_structural_mass + empennage_struct_cg_x * empennage_mass + nsm_cg_x * nsm_mass)/ total_mass) #* 0.96
        cg_y = ( wing_boom_fuselage_structural_cg_y * wing_boom_fuselage_structural_mass + empennage_struct_cg_y * empennage_mass + nsm_cg_y * nsm_mass)/ total_mass
        cg_z = ( wing_boom_fuselage_structural_cg_z * wing_boom_fuselage_structural_mass + empennage_struct_cg_z * empennage_mass + nsm_cg_z * nsm_mass)/ total_mass
        
        # Inertia tensor can be added because they're taken about the same reference point (aircraft cg)
        Ixx = wing_boom_fuselage_structural_I_xx_global + empennage_struct_I_xx_global + nsm_I_xx_global
        Iyy = wing_boom_fuselage_structural_I_yy_global + empennage_struct_I_yy_global + nsm_I_yy_global
        Izz = wing_boom_fuselage_structural_I_zz_global + empennage_struct_I_zz_global + nsm_I_zz_global
        Ixy = wing_boom_fuselage_structural_I_xy_global + empennage_struct_I_xy_global + nsm_I_xy_global
        Ixz = wing_boom_fuselage_structural_I_xz_global + empennage_struct_I_xz_global + nsm_I_xz_global
        Iyz = wing_boom_fuselage_structural_I_yz_global + empennage_struct_I_yz_global + nsm_I_yz_global


        self.register_module_output('total_structural_mass',(empennage_mass +  wing_boom_fuselage_structural_mass) * 1.22)
        self.register_module_output('non_structural_mass', nsm_mass*1)

        self.register_module_output(
            name='mass', 
            var=total_mass)
        self.register_module_output(
            name='cgx', 
            var=cg_x)
        self.register_module_output(
            name='cgy',  
            var=cg_y)
        self.register_module_output(
            name='cgz', 
            var=cg_z)
        
        cg_vector = self.register_module_output('cg_vector', shape=(3, ), val=0)
        cg_vector[0] = cg_x
        cg_vector[1] = cg_y
        cg_vector[2] = cg_z

        inertia_tensor = self.register_module_output('inertia_tensor', shape=(3, 3), val=0)
        inertia_tensor[0, 0] = csdl.reshape(Ixx, (1, 1)) 
        inertia_tensor[1, 1] = csdl.reshape(Iyy, (1, 1)) 
        inertia_tensor[2, 2] = csdl.reshape(Izz, (1, 1)) 
        inertia_tensor[0, 2] = csdl.reshape(Ixz, (1, 1)) 
        inertia_tensor[2, 0] = csdl.reshape(Ixz, (1, 1)) 

        self.register_module_output(
            name='ixx', 
            var=Ixx)
        self.register_module_output(
            name='iyy', 
            var=Iyy)
        self.register_module_output(
            name='izz', 
            var=Izz)
        self.register_module_output(
            name='ixz',  
            var=Ixz)
        
        # self.register_output('Ixy', Ixy)
        # self.register_output('Iyz', Iyz)
