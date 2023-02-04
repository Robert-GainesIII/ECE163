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
        self.state = self.dynamicsModel.getVehicleState()
        self.dot = self.dynamicsModel.getVehicleDerivative()
        self.dT = self.dynamicsModel.dT

        self.initialHeight = initialHeight
        self.initialSpeed = initialSpeed


    def reset(self):
        pass
    
    def getVehicleState(self):
        pass

    def setVehicleState(self, state):
        self.dynamicsModel.setVehicleState(state)
        

    def getVehicleDynamicsModel(self):
        pass

    def update(self):
        pass

    def CalculateCoeff_alpha(self):
        pass

    def aeroForces(self):
        pass

    def controlForces(self):
        pass

    def gravityForces(self):
        pass

    def CalculatePropForces(self):
        pass

    def updateForces(self):
        pass