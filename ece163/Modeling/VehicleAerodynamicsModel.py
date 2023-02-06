"""
Engineer: Robert Gaines
File Name : VehicleAerodynamicsModel.py
Purpose : Contains functions to compute aerodynamic forces and moments acting on the MAV given certain control surface inputs
"""

import math
from ..Containers import States
from ..Containers import Inputs
from ..Modeling import VehicleDynamicsModel
from ..Modeling import WindModel
from ..Utilities import MatrixMath
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC
from matplotlib import pyplot as plt
import numpy as np

def cl_attached(alpha):
    print("calculating cl attached")
    x = VPC.Cl0 + VPC.CLalpha * alpha
    print("cl_attached= " + str(x))
    return x

def cd_attached(alpha):
    print("calculating cd attached")
    x = VPC.CDp + (math.pow((VPC.CL0 + VPC.CLalpha * alpha),2))/(math.pi * math.e * VPC.AR)
    print("cd_attached= " + str(x))
    return x

def cl_seperated(alpha):
    print("calculating cl seperated ")
    x = 2 *math.sin(alpha) * math.cos(alpha)
    print("cl_seperated= " + str(x))
    return x

def cd_seperated(alpha):
    print("calculating cd seperated ")
    x = 2 * math.pow(math.sin(alpha), 2)
    print("cd_seperated =" + str(x))
    return x

def Cl_fromA(alpha):
                                #not sure what we should pass for M, do i need to calculate this?
  
    sigmasss = (alpha, VPC.alpha0, 0.5)
    x = (1 - sigmasss) * cl_attached(alpha) + sigmasss*cd_seperated(alpha)
    print (x)
    return  x

def Cd_fromA(alpha):
                                #not sure what we should pass for M, do i need to calculate this?
    sigmasss = (alpha, VPC.alpha0, 0.5)
    print(sigmasss)
    a = cd_attached(alpha)
    s = cl_seperated(alpha)
    x = ( 1 - sigmasss )*  a + sigmasss * s
    print(x)    
    return x

#blending function

def sigma(a, a0, M):
    num = (1 + np.exp(-M*(a-a0)) + np.exp(M*(a+a0)))
    den = (1 + np.exp(-M*(a-a0)))*(1 + np.exp(M*(a+a0)))
    return num/den

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
        print("hellooooo")
        Cd_alpha = Cd_fromA(alpha)  
        print("test2")
        Cl_alpha = Cl_fromA(alpha)
        print("t3")
        Cm_alpha = VPC.CM0 + VPC.CMalpha * alpha
        print("returning CdAlpha = " + Cd_alpha)
        print("returning ClAlpha = " + Cl_alpha)
        print("returning CmAlpha = " + Cm_alpha)
        return Cd_alpha, Cl_alpha, Cm_alpha



    def aeroForces(self, state):
        pass

    def controlForces(self, state, controls):
        pass

    def gravityForces(self, state):

        forcesnMoments = Inputs.forcesMoments()
        forceVector = [[forcesnMoments.Fx],[forcesnMoments.Fy],[VPC.g0*VPC.mass]]
        gravityInBody = MatrixMath.multiply(state.R, forceVector)

        forcesnMoments.Fx = gravityInBody[0][0]
        forcesnMoments.Fy = gravityInBody[1][0]
        forcesnMoments.Fz = gravityInBody[2][0]

        return forcesnMoments

    def CalculatePropForces(self, Va, Throttle):
        pass

    def updateForces(self, state, controls, wind=None):
        pass