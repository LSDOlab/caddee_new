from caddee.utils.caddee_base import CADDEEBase


class AtmosphereModel(CADDEEBase): pass

class SimpleAtmosphereModel(AtmosphereModel):
    def initialize(self, kwargs): pass 

    def _assemble_csdl(self, design_condition): 
        # from caddee.csdl_core.system_model_csdl.design_scenario_csdl.atmosphere_csdl.atmosphere_csdl import SimpleAtmosphereCSDL
        from caddee.csdl_core_modules.system_model_csdl.design_scenario_csdl.atmosphere_csdl.atmosphere_csdl import SimpleAtmosphereCSDL

        csdl_model = SimpleAtmosphereCSDL(
            design_condition=design_condition,
            atmosphere_model=self,
            
        )

        return csdl_model
        