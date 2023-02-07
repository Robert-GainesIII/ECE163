"""
Engineer: Robert Gaines III
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
    x = VPC.CL0 + VPC.CLalpha * alpha
    print("cl_attached= " + str(x))
    return x

def cd_attached(alpha):
    print("calculating cd attached")
    x = VPC.CDp + (math.pow((VPC.CL0 + VPC.CLalpha * alpha),2))/(math.pi * VPC.e* VPC.AR)
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
  
    sigmasss = sigma(alpha, VPC.alpha0, VPC.M)
    x = (1 - sigmasss) * cl_attached(alpha) + sigmasss*cl_seperated(alpha)
    print (x)
    return  x

def Cd_fromA(alpha):
                                #not sure what we should pass for M, do i need to calculate this?
    sigmasss = sigma(alpha, VPC.alpha0, VPC.M)
    print(sigmasss)
    a = cd_attached(alpha)
    s = cd_seperated(alpha)
    x = ( 1 - sigmasss )*  a + sigmasss * s
    print(x)    
    return x

#blending function

def sigma(a, a0, M):
    num = (1 + np.exp(-M*(a-a0)) + np.exp(M*(a+a0)))
    den = (1 + np.exp(-M*(a-a0)))*(1 + np.exp(M*(a+a0)))
    return num/den

#EQUATION 4.6
def calcLiftForce(state):
    Va = math.hypot(state.u, state.v, state.w)
    alpha = math.tanh(state.w/state.u)
    if math.isclose(Va, 0.0):
        return 0
    else:
        c_2Va = 0.5 * VPC.c * state.q / Va
        b_2Va = 0.5 * VPC.b / Va

    fLiftTerm1 = 1/2* VPC.rho * math.pow(Va, 2) * VPC.S
    fLiftTerm2 = Cl_fromA(alpha)
    fLiftTerm3 = (VPC.CLq * c_2Va)
    return fLiftTerm1 * (fLiftTerm2 + fLiftTerm3)
#EQUATION 4.7
def calcDragForce(state):
    Va = math.hypot(state.u, state.v, state.w)
    alpha = math.tanh(state.w/state.u)
    if math.isclose(Va, 0.0):
        c_2Va = 1.0
        b_2Va = 1.0
    else:
        c_2Va = 0.5 * VPC.c * state.q / Va
        b_2Va = 0.5 * VPC.b / Va
    fLiftTerm1 = 1/2* VPC.rho * math.pow(Va, 2) * VPC.S
    fLiftTerm2 = Cd_fromA(alpha)
    fLiftTerm3 = (VPC.CDq * c_2Va)
    return fLiftTerm1 * (fLiftTerm2 + fLiftTerm3)
#EQUATION 4.5
def calcMoment(state):
    Va = math.hypot(state.u, state.v, state.w)
    alpha = math.tanh(state.w/state.u)
    if math.isclose(Va, 0.0):
        c_2Va = 1.0
        b_2Va = 1.0
    else:
        c_2Va = 0.5 * VPC.c * state.q / Va
        b_2Va = 0.5 * VPC.b / Va
    fLiftTerm1 = 1/2* VPC.rho * math.pow(Va, 2) * VPC.S * VPC.c
    fLiftTerm2 = VPC.CM0
    fLiftTerm3 = VPC.CMalpha * alpha
    fLiftTerm4 = (VPC.CMq* c_2Va)
    return fLiftTerm1 * (fLiftTerm2 + fLiftTerm3 + fLiftTerm4)

#EQUATION 4.14
def calcFy(state):
    

    if math.isclose(Va, 0.0):
        c_2Va = 1.0
        b_2Va = 1.0
    else:
        c_2Va = 0.5 * VPC.c * state.q / Va
        b_2Va = 0.5 * VPC.b / Va
    fLiftTerm1 = 1/2* VPC.rho * math.pow(Va, 2) * VPC.S
    fLiftTerm2 = VPC.CY0
    fLiftTerm3 = VPC.CYbeta * slideslip
    fLiftTerm4 = (VPC.CYp* b_2Va* state.p)
    fLiftTerm5 = (VPC.CYr* b_2Va * state.r)
    return fLiftTerm1 * (fLiftTerm2 + fLiftTerm3 + fLiftTerm4 + fLiftTerm5)

#EQUATION 4.15
def calcMomentL(state):
    Va = math.hypot(state.u, state.v, state.w)
    alpha = math.tanh(state.w/state.u)
    slideslip = math.sinh(state.v/Va)

    if math.isclose(Va, 0.0):
        c_2Va = 1.0
        b_2Va = 1.0
    else:
        c_2Va = 0.5 * VPC.c * state.q / Va
        b_2Va = 0.5 * VPC.b / Va
    fLiftTerm1 = 1/2* VPC.rho * math.pow(Va, 2) * VPC.S * VPC.b
    fLiftTerm2 = VPC.Cl0
    fLiftTerm3 = VPC.Clbeta * slideslip
    fLiftTerm4 = (VPC.Clp* b_2Va * state.p)
    fLiftTerm5 = (VPC.Clr* b_2Va * state.r)
    return fLiftTerm1 * (fLiftTerm2 + fLiftTerm3 + fLiftTerm4 + fLiftTerm5)

#EQUATION 4.16
def calcMomentN(state):
    Va = math.hypot(state.u, state.v, state.w)
    alpha = math.tanh(state.w/state.u)
    slideslip = math.sinh(state.v/Va)

    if math.isclose(Va, 0.0):
        c_2Va = 1.0
        b_2Va = 1.0
    else:
        c_2Va = 0.5 * VPC.c * state.q / Va
        b_2Va = 0.5 * VPC.b / Va

    fLiftTerm1 = 1/2* VPC.rho * math.pow(Va, 2) * VPC.S * VPC.b
    fLiftTerm2 = VPC.Cn0
    fLiftTerm3 = VPC.Cnbeta * slideslip
    fLiftTerm4 = (VPC.Cnp* VPC.b * state.p)
    fLiftTerm5 = (VPC.Cnr* VPC.b * state.r)
    return fLiftTerm1 * (fLiftTerm2 + fLiftTerm3 + fLiftTerm4 + fLiftTerm5)

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

    def Update(self, controls):

        self.state = self.getVehicleState()
        

    #:param alpha, Angle of Attack [rad]
    # return: 
    #Coefficient of Lift, CL_alpha (unitless), Coefficient of Drag,
    #CD_alpha (unitless), Coefficoent of Moment, CM_alpha (unitless)
    def CalculateCoeff_alpha(self, alpha):
        Cd_alpha = Cd_fromA(alpha)  
        Cl_alpha = Cl_fromA(alpha)
        Cm_alpha = VPC.CM0 + VPC.CMalpha * alpha
        print("returning CdAlpha = " + str(Cd_alpha))
        print("returning ClAlpha = " + str(Cl_alpha))
        print("returning CmAlpha = " + str(Cm_alpha))
        return Cl_alpha, Cd_alpha, Cm_alpha


    #you’ll be calculating the first two terms of Beard 4.6 and 4.7, the first three terms
    #of Beard 4.5, and the first four terms of Beard 4.14, 4.15, and 4.16.
    
    def aeroForces(self, state):
        print("start aeroForces()")
        forcesnMoments = Inputs.forcesMoments()
        
        print("calculated beta now for actaul function")
        fl = calcLiftForce(state)
        print("end aeroForces")
        return forcesnMoments

    def controlForces(self, state, controls):
        forcesnMoments = Inputs.forcesMoments()

        return forcesnMoments

    def gravityForces(self, state):

        forcesnMoments = Inputs.forcesMoments()
        forceVector = [[forcesnMoments.Fx],[forcesnMoments.Fy],[VPC.g0*VPC.mass]]
        gravityInBody = MatrixMath.multiply(state.R, forceVector)

        forcesnMoments.Fx = gravityInBody[0][0]
        forcesnMoments.Fy = gravityInBody[1][0]
        forcesnMoments.Fz = gravityInBody[2][0]

        return forcesnMoments

    def CalculatePropForces(self, Va, Throttle):
        J =  (2* math.pi * Va)/Throttle*VPC.D_prop
        Ct = VPC.C_T0 + VPC.C_T1 * J + VPC.C_T2 * math.pow(J,2)
        Cq = VPC.C_Q0 + VPC.C_Q1 * J + VPC.C_Q2 * math.pow(J,2)
        
        Fprop = (VPC.rho * math.pow(Throttle, 2) * math.pow(VPC.D_prop, 4) * Ct)/(4*math.pow(math.pi,2))
        Mprop = (-1 * VPC.rho * math.pow(Throttle, 2) * math.pow(VPC.D_prop, 5) * Cq)/(4*math.pow(math.pi,2))
        
        return (Fprop, Mprop)

    def updateForces(self, state, controls, wind=None):
        forcesnMoments = Inputs.forcesMoments()
        state.Va = math.hypot(state.u, state.v, state.w)
        state.alpha = math.atan2(state.w,state.u)
        if(math.isclose(state.Va, 0)):
            state.beta = math.asin(state.v/math.hypot(state.u, state.v, state.w))
        else:
            state.beta = 0
        return forcesnMoments