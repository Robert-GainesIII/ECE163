import math
from ..Containers import States
from ..Utilities import MatrixMath
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC

class VehicleDynamicsModel:
    
    def __init__(self, dT = VPC.dT):
        self.state = States.vehicleState()
        self.dot = 0
        self.dT = dT

    def ForwardEuler(self, dT, state, dot):
        newState = States.vehicleState()
        return newState
    
    def IntegrateState(self, dT, state, dot):
        newState = States.vehicleState()
        return newState

    def Rexp(self, dT, state, dot):
        rexp = 0
        return rexp

    def Update(forcesMoments):

        print("start of update")

        print("end of update.")