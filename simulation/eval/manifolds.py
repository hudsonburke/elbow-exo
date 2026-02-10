import mujoco
import numpy as np

"""
The goal of this module is to provide functions for evaluating manipulators/exoskeletons in terms of their configuration space and task space. This will allow us to compare different designs and control strategies in a more systematic way.

Inputs:
- problem with a goal and constraints by which that problem can be solved
- manipulator/exoskeleton model with specs / constraints (i.e. torque limits, joint limits, etc.)

Methods:
- Compute problem manifold
- Compute configuration space for model
- 

Start with just position and then figure out how to extend to velocity and acceleration.

This is kind of like policy search on manifolds?
"""


def calc_config_space():
    pass


def calc_task_space():
    pass
