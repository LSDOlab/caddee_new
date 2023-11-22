from dataclasses import dataclass
import m3l


@dataclass
class MassProperties:
    mass : m3l.Variable
    cg_vector : m3l.Variable
    inertia_tensor : m3l.Variable


