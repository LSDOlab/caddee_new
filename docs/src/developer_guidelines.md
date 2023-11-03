# Developer guidelines

In this sectio we give examples for developers to connect their solvers to CADDEE. In this context the terms 'solver' refers to a piece of python code that implements a mathematical model. Examples are aerodynamic or propulsive sovlers, which predict quantities such as lift, drag, or thrust. We will provide information about what is required for a solver to be integrated into CADDEE. 

## Solver requirements

### CSDL
Since CADDEE leverages large-scale gradient-based sensitivity analysis, any solver should provide ways to compute derivatives. For the current version of CADDEE, this requirement must be met by implementing a sovler in the Computational System Design Language [CSDL](https://lsdolab.github.io/csdl/), which is a domain-embedded modeling language for multidisciplinary design optimization (MDO), which automates derivative computation. Future versions of CADDEE will support the integration of solvers written in any automatic differentation (AD) package.  

### M3L
In addition, a solver's front-end must be wrapped witin the multi-fidelity, multidisciplinary modeling language [M3L](https://github.com/LSDOlab/m3l), which is a package for modularly specifying model data transfer. 


### Specifying solver outputs 
Specifying solver outputs must be specified through the use of dataclasses, where each attribute is an M3L variable.

## Simple examples