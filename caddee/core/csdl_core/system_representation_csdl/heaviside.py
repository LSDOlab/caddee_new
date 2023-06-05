import csdl
from csdl_om import Simulator
import numpy as np

# class PiecewiseHeavisideOperation(csdl.CustomExplicitOperation):
#     def initialize(self):
#         self.parameters.declare('eps')
#         self.parameters.declare('input_name')
#         self.parameters.declare('output_name')
#     def define(self):
#         self.add_input(self.parameters['input_name'])
#         self.add_output(self.parameters['output_name'])

#         self.declare_derivatives(self.parameters['output_name'], self.parameters['input_name'])

#     def compute(self, inputs, outputs):
#         eps = self.parameters['eps']
#         input = self.parameters['input_name']
#         output = self.parameters['output_name']
#         if inputs[input] < -eps:
#             outputs[output] = 0
#         elif inputs[input] > eps:
#             outputs[output] = 1
#         else:
#             outputs[output] = -1/4*(inputs[input]/eps)**3+3/4*(inputs[input]/eps)+1/2
        
#     def compute_derivatives(self, inputs, derivatives):
#         eps = self.parameters['eps']
#         input = self.parameters['input_name']
#         output = self.parameters['output_name']
#         derivatives[output, input] = -3/4*(inputs[input]/eps)**2/eps+3/4/eps

# class PiecewiseHeavisideModel(csdl.Model):
#     def initialize(self):
#         self.parameters.declare('shape')
#         self.parameters.declare('eps')
    
#     def define(self):
#         shape = self.parameters['shape']
#         eps = self.parameters['eps']
#         input_csdl = self.declare_variable('input', shape=shape)
#         output_csdl = self.create_output('output', shape=shape)
#         for i in range(shape[0]):
#             op = PiecewiseHeavisideOperation(eps=eps, input_name = input_csdl[i].name, output_name = output_csdl[i].name)
#             output_csdl[i] = csdl.custom(input_csdl[i], op=op)

class PiecewiseHeavisideOperation(csdl.CustomExplicitOperation):
    def initialize(self):
        self.parameters.declare('eps')
        self.parameters.declare('shape')

    def define(self):
        self.add_input('input', shape=self.parameters['shape'])
        self.add_output('output', shape=self.parameters['shape'])

        self.declare_derivatives('output', 'input')

    def compute(self, inputs, outputs):
        eps = self.parameters['eps']
        shape = self.parameters['shape']
        input = inputs['input']
        for i in range(shape[0]):
            if input[i] < -eps:
                outputs['output'][i] = 0
            elif input[i] > eps:
                outputs['output'][i] = 1
            else:
                outputs['output'][i] = -1/4*(inputs[input]/eps)**3+3/4*(inputs[input]/eps)+1/2
        
    def compute_derivatives(self, inputs, derivatives):
        eps = self.parameters['eps']
        shape = self.parameters['shape']
        input = inputs['input']
        derivative = np.zeros((shape[0],shape[0]))
        for i in range(shape[0]):
            if inputs[input[i]] >= -eps and inputs[input[i]] <= eps:
                derivative[i,i] = -1/4*(input[i]/eps)**3+3/4*(input[i]/eps)+1/2
        derivatives['output', 'input'] = derivatives

class PiecewiseHeavisideModel(csdl.Model):
    def initialize(self):
        self.parameters.declare('shape')
        self.parameters.declare('eps')
    
    def define(self):
        shape = self.parameters['shape']
        eps = self.parameters['eps']
        input_csdl = self.declare_variable('input', shape=shape)
        op = PiecewiseHeavisideOperation(eps=eps, shape=shape)
        output_csdl = csdl.custom(input_csdl, op=op)
        self.register_output('output', output_csdl)