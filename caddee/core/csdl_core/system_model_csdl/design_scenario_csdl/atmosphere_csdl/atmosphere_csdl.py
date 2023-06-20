from caddee.utils.base_model_csdl import BaseModelCSDL
from lsdo_modules.module_csdl.module_csdl import ModuleCSDL
from caddee.core.caddee_core.system_model.design_scenario.design_condition.atmosphere.atmosphere import SimpleAtmosphereModel
from csdl import Model

"""
class SimpleAtmosphereCSDL(ModuleCSDL):
    def initialize(self):
        self.parameters.declare('atmosphere_model', types=SimpleAtmosphereModel)
        self.parameters.declare('design_condition')

    def define(self):
        # atmosphere_model = self.parameters['atmosphere_model']
        prefix = self.parameters['design_condition']
        
        h = self.declare_variable(prefix + '_' + 'altitude', shape=(1, )) * 1e-3 # Convert to km
        # Constants
        L = 6.5 # K/km
        R = 287
        T0 = 288.16
        P0 = 101325
        g0 = 9.81
        mu0 = 1.735e-5
        S1 = 110.4
        gamma = 1.4

        # Temperature 
        T = - h * L + T0

        # Pressure 
        P = P0 * (T/T0)**(g0/(L * 1e-3)/R)
        
        # Density
        rho = P/R/T
        # self.print_var(rho)
        
        # Dynamic viscosity (using Sutherland's law)  
        mu = mu0 * (T/T0)**(3/2) * (T0 + S1)/(T + S1)

        # speed of sound 
        a = (gamma * R * T)**0.5

        self.register_output(prefix + '_' + 'temperature',T)
        self.register_output(prefix + '_' + 'pressure',P)
        self.register_output(prefix + '_' + 'density',rho)
        self.register_output(prefix + '_' + 'dynamic_viscosity',mu)
        self.register_output(prefix + '_' + 'speed_of_sound', a)

"""

import csdl
import numpy as np


g = 9.806 # m/(s^2)
Ts = 288.16 # deg K @ sea level
Ps = 1.01325E5 # Pascals at sea level
rhoS = 1.225 # kg/m^3 at sea level
R = 287 # J/(Kg-K) gas constant
P11 = 2.2629E04 # pressure @ 11km
P25 = 2.4879E03 # pressure @ 25km
rho11 = 0.3639 # density @ 11km
rho25 = 0.0400 # density @ 25km
mu0 = 1.735e-5
S1 = 110.4
gamma = 1.4


class SimpleAtmosphereCSDL(ModuleCSDL):
    def initialize(self):
        self.parameters.declare('atmosphere_model', types=SimpleAtmosphereModel)
        self.parameters.declare('design_condition')



        num = 500
        altitude = np.linspace(0,47000,num)

        pressure = np.zeros(num)
        density = np.zeros(num)
        temperature = np.zeros(num)

        # standard atmosphere model
        for i, z in enumerate(altitude):
            # valid through 47000 m / 154000 ft
            if z <= 11000:
                a = -6.5E-3 # K/m
                temperature[i] = Ts + a*z
                pressure[i] = Ps*((temperature[i]/Ts)**((-g)/(a*R)))
                density[i] = rhoS*((temperature[i]/Ts)**(-((g/(a*R)) + 1)))
            elif z > 11000 and z <= 25000:
                temperature[i] = 216.6 # isothermal region
                pressure[i] = P11*(np.exp(-(g/(R*temperature[i]))*(z - 11000)))
                density[i] = rho11*(np.exp(-(g/(R*216.66))*(z - 11000)))
            elif z > 25000 and z <= 47000:
                a = 3E-3
                temperature[i] = 216.66 + a*(z - 25000)
                pressure[i] = P25*((temperature[i]/216.66)**((-g)/(a*R)))
                density[i] = rho25*((temperature[i]/216.66)**(-((g/(a*R)) + 1)))


        degree = 6
        density_coefs = np.polyfit(altitude, density, degree)
        pressure_coefs = np.polyfit(altitude, pressure, degree)
        temperature_coefs = np.polyfit(altitude, temperature, degree)
        self.density_coefs = density_coefs
        self.pressure_coefs = pressure_coefs
        self.temperature_coefs = temperature_coefs

        """
        d_model = np.polyval(density_coefs, altitude)
        p_model = np.polyval(pressure_coefs, altitude)
        t_model = np.polyval(temperature_coefs, altitude)
        plt.plot(d_model)
        plt.show()
        plt.plot(p_model)
        plt.show()
        plt.plot(t_model)
        plt.show()
        """

    def define(self):
        prefix = self.parameters['design_condition']


        pd = self.density_coefs
        pp = self.pressure_coefs
        pt = self.temperature_coefs
        N = len(pd)

        # x = self.declare_variable('altitude',val=1000)
        x = self.declare_variable(prefix + '_' + 'altitude', shape=(1, ))

        #px = p[0]*x**(N-1) + p[1]*x**(N-2) + p[2]*x**(N-3) + p[3]*x**(N-4) + p[4]*x**(N-5)...
        temp_density = self.create_output('temp_density',shape=(N),val=0)
        temp_pressure = self.create_output('temp_pressure',shape=(N),val=0)
        temp_temperature = self.create_output('temp_temperature',shape=(N),val=0)
        for i in range(N):
            temp_density[i] = pd[i]*x**(N - 1 - i)
            temp_pressure[i] = pp[i]*x**(N - 1 - i)
            temp_temperature[i] = pt[i]*x**(N - 1 - i)

        
        density = csdl.sum(temp_density)
        #self.register_output('density',density)
        pressure = csdl.sum(temp_pressure)
        #self.register_output('pressure',pressure)
        temperature = csdl.sum(temp_temperature)
        #self.register_output('temperature',temperature)
        

        # dynamic viscosity
        mu = mu0 * (temperature/Ts)**(3/2) * (Ts + S1)/(temperature + S1)

        # speed of sound 
        a = (gamma * R * temperature)**0.5

        # caddee outputs:
        self.register_output(prefix + '_' + 'temperature',temperature)
        self.register_output(prefix + '_' + 'pressure',pressure)
        self.register_output(prefix + '_' + 'density',density)
        self.register_output(prefix + '_' + 'dynamic_viscosity',mu)
        self.register_output(prefix + '_' + 'speed_of_sound', a)