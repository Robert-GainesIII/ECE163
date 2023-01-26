import math
from ..Containers import States
from ..Utilities import MatrixMath
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC

class VehicleDynamicsModel:
    
    def __init__(self, dT = VPC.dT):
        self.state = States.vehicleState()
        self.dot = States.vehicleState()
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

    def Update(self,forcesnmoments):

        print("start of update")

        print("end of update.")

    def derivative(self,state, forcesnmoments):
        #compute the derivative of the given state with the provided forces n moments then return 
        #an updated state where the derivatives replace what they dervied from i.e pqr => PdotQdotRdot

        return state

    def getVehicleDerivative(self):

        return self.state

    def getVehicleState(self):

        return self.state
    
    def reset(self):

        newState = States.vehicleState()
        self.state = newState
    
    def setVehicleDot(self, dot):

        dot = self.dot

    def setVehicleState(self, state):

        state = self.state
        