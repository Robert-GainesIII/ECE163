import math
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Linearized
from ece163.Utilities import MatrixMath


def CreateTransferFunction(trimState, trimInputs):

    mytf = Linearized.transferFunctions()


    return mytf

def dThrust_dThrottle(Va, Throttle, epsilon =0.01):
    
    #partial derivative {N/PWM}
    dt_dDeltaT = 0

    return dt_dDeltaT


def dThrust_dVa(Va, Throttle, epsilon=0.5):
    #partial derivative {N-s/m}
    dt_dVa = 0

    return dt_dVa