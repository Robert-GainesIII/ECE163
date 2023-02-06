import math
from ..Containers import States
from ..Containers import Inputs
from ..Modeling import VehicleDynamicsModel
from ..Modeling import WindModel
from ..Utilities import MatrixMath
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC


class VehicleAerodynamicsModel:
    
    def __init__(self, initialSpeed = VPC.InitialSpeed, initialHeight = VPC.InitialDownPosition):
        #create class instance of vehicle dynamics
        self.dynamicsModel = VehicleDynamicsModel.VehicleDynamicsModel()
        print(self.dynamicsModel)
        self.state = self.dynamicsModel.getVehicleState()
        self.dot = self.dynamicsModel.getVehicleDerivative()
        self.dT = self.dynamicsModel.dT

        self.initialHeight = initialHeight
        self.initialSpeed = initialSpeed


    def reset(self):
        pass
    
    def getVehicleState(self):
        return self.dynamicsModel.getVehicleState()

    def setVehicleState(self, state):
        self.dynamicsModel.setVehicleState(state)
        return

    def getVehicleDynamicsModel(self):
        
        return self.dynamicsModel

    def update(self):

        self.state = self.getVehicleState()
        

    #:param alpha, Angle of Attack [rad]
    # return: 
    #Coefficient of Lift, CL_alpha (unitless), Coefficient of Drag,
    #CD_alpha (unitless), Coefficoent of Moment, CM_alpha (unitless)
    def CalculateCoeff_alpha(self, alpha):
        pass

    def aeroForces(self, state):
        pass

    def controlForces(self, state, controls):
        pass

    def gravityForces(self, state):

        forcesnMoments = Inputs.forcesMoments()
        forceVector = [[forcesnMoments.Fx],[forcesnMoments.Fy],[9.8*VPC.mass]]
        gravityInBody = MatrixMath.multiply(state.R, forceVector)

        forcesnMoments.Fx = gravityInBody[0][0]
        forcesnMoments.Fy = gravityInBody[1][0]
        forcesnMoments.Fz = gravityInBody[2][0]

        return forcesnMoments

    def CalculatePropForces(self, Va, Throttle):
        pass

    def updateForces(self, state, controls, wind=None):
        pass