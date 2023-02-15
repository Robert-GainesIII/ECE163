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
    #print("calculating cl attached")
    x = VPC.CL0 + VPC.CLalpha * alpha
    #print("cl_attached= " + str(x))
    return x

def cd_attached(alpha):
    #print("calculating cd attached")
    x = VPC.CDp + (math.pow((VPC.CL0 + VPC.CLalpha * alpha),2))/(math.pi * VPC.e* VPC.AR)
    #print("cd_attached= " + str(x))
    return x

def cl_seperated(alpha):
    #print("calculating cl seperated ")
    x = 2 *math.sin(alpha) * math.cos(alpha)
    #print("cl_seperated= " + str(x))
    return x

def cd_seperated(alpha):
    #print("calculating cd seperated ")
    x = 2 * math.pow(math.sin(alpha), 2)
    #print("cd_seperated =" + str(x))
    return x

def Cl_fromA(alpha):
                                #not sure what we should pass for M, do i need to calculate this?
  
    sigmasss = sigma(alpha, VPC.alpha0, VPC.M)
    x = (1 - sigmasss) * cl_attached(alpha) + sigmasss*cl_seperated(alpha)
    #print (x)
    return  x

def Cd_fromA(alpha):
                                #not sure what we should pass for M, do i need to calculate this?
    sigmasss = sigma(alpha, VPC.alpha0, VPC.M)
    #print(sigmasss)
    a = cd_attached(alpha)
    s = cd_seperated(alpha)
    x = ( 1 - sigmasss )*  a + sigmasss * s
    #print(x)    
    return x

#blending function

def sigma(a, a0, M):
    num = (1 + np.exp(-M*(a-a0)) + np.exp(M*(a+a0)))
    den = (1 + np.exp(-M*(a-a0)))*(1 + np.exp(M*(a+a0)))
    return num/den

#EQUATION 4.6
def calcLiftForce(state):
   
  
    
    c_2Va = 0.5 * VPC.c * state.q / state.Va
     

    fLiftTerm1 = 1/2* VPC.rho * math.pow(state.Va, 2) * VPC.S
    fLiftTerm2 = Cl_fromA(state.alpha)
    fLiftTerm3 = (VPC.CLq * c_2Va)
    return fLiftTerm1 * (fLiftTerm2 + fLiftTerm3)
#EQUATION 4.7
def calcDragForce(state):
  
    c_2Va = 0.5 * VPC.c * state.q / state.Va
    
    fLiftTerm1 = 1/2* VPC.rho * math.pow(state.Va, 2) * VPC.S
    fLiftTerm2 = Cd_fromA(state.alpha)
    fLiftTerm3 = (VPC.CDq * c_2Va)
    return fLiftTerm1 * (fLiftTerm2 + fLiftTerm3)
#EQUATION 4.5
def calcMoment(state):
   
    c_2Va = 0.5 * VPC.c * state.q / state.Va

    fLiftTerm1 = 1/2* VPC.rho * math.pow(state.Va, 2) * VPC.S * VPC.c
    fLiftTerm2 = VPC.CM0
    fLiftTerm3 = VPC.CMalpha * state.alpha
    fLiftTerm4 = (VPC.CMq* c_2Va)
    return fLiftTerm1 * (fLiftTerm2 + fLiftTerm3 + fLiftTerm4)

#EQUATION 4.14
def calcFy(state):
    
    b_2Va = 0.5 * VPC.b / state.Va
    fLiftTerm1 = 1/2* VPC.rho * math.pow(state.Va, 2) * VPC.S
    fLiftTerm2 = VPC.CY0
    fLiftTerm3 = VPC.CYbeta * state.beta
    fLiftTerm4 = (VPC.CYp* b_2Va* state.p)
    fLiftTerm5 = (VPC.CYr* b_2Va * state.r)
    return fLiftTerm1 * (fLiftTerm2 + fLiftTerm3 + fLiftTerm4 + fLiftTerm5)

#EQUATION 4.15
def calcMomentL(state):
    
    b_2Va = 0.5 * VPC.b / state.Va
    fLiftTerm1 = 1/2* VPC.rho * math.pow(state.Va, 2) * VPC.S * VPC.b
    fLiftTerm2 = VPC.Cl0
    fLiftTerm3 = VPC.Clbeta * state.beta
    fLiftTerm4 = (VPC.Clp* b_2Va * state.p)
    fLiftTerm5 = (VPC.Clr* b_2Va * state.r)
    return fLiftTerm1 * (fLiftTerm2 + fLiftTerm3 + fLiftTerm4 + fLiftTerm5)

#EQUATION 4.16
def calcMomentN(state):
    
    b_2Va = 0.5 * VPC.b / state.Va

    fLiftTerm1 = 1/2* VPC.rho * math.pow(state.Va, 2) * VPC.S * VPC.b
    fLiftTerm2 = VPC.Cn0
    fLiftTerm3 = VPC.Cnbeta * state.beta
    fLiftTerm4 = (VPC.Cnp* b_2Va * state.p)
    fLiftTerm5 = (VPC.Cnr* b_2Va * state.r)
    return fLiftTerm1 * (fLiftTerm2 + fLiftTerm3 + fLiftTerm4 + fLiftTerm5)

class VehicleAerodynamicsModel:
    
    def __init__(self, initialSpeed = VPC.InitialSpeed, initialHeight = VPC.InitialDownPosition):
        #create class instance of vehicle dynamics
        self.dynamicsModel = VehicleDynamicsModel.VehicleDynamicsModel()
        self.myWindModel = WindModel.WindModel()
        #print(self.dynamicsModel)
        self.state = self.dynamicsModel.getVehicleState()
        self.dot = self.dynamicsModel.getVehicleDerivative()
        self.dT = self.dynamicsModel.dT
        self.dynamicsModel.pe = VPC.InitialEastPosition
        self.dynamicsModel.pn = VPC.InitialNorthPosition
        self.dynamicsModel.pd = VPC.InitialDownPosition
        self.dynamicsModel.u = VPC.InitialSpeed

        self.initialHeight = initialHeight
        self.initialSpeed = initialSpeed


    def reset(self):
        self.dynamicsModel.reset()
        self.myWindModel.reset()
        #print(self.dynamicsModel)
        self.state = self.dynamicsModel.getVehicleState()
        self.dot = self.dynamicsModel.getVehicleDerivative()
        self.dT = self.dynamicsModel.dT
        self.dynamicsModel.pe = VPC.InitialEastPosition
        self.dynamicsModel.pn = VPC.InitialNorthPosition
        self.dynamicsModel.pd = VPC.InitialDownPosition
        self.dynamicsModel.u = VPC.InitialSpeed

        self.initialHeight =  VPC.InitialDownPosition
        self.initialSpeed = VPC.initialSpeed
    
    def getVehicleState(self):
        return self.dynamicsModel.state

    def setVehicleState(self, state):
        self.dynamicsModel.state = state
        return

    def getVehicleDynamicsModel(self):
        
        return self.dynamicsModel

    def Update(self, controls):

        DIYAISCRAZY = self.dynamicsModel.getVehicleState()
        self.myWindModel.Update()
        f = self.updateForces(DIYAISCRAZY, controls, self.myWindModel.getWind())
        self.dynamicsModel.Update(f)
        '''
        self.myWindModel.Update()
        airspeed = self.CalculateAirspeed(DIYAISCRAZY, self.myWindModel)
        self.dynamicsModel.state.Va = airspeed[0]
        self.dynamicsModel.state.alpha = airspeed[1]
        self.dynamicsModel.state.beta = airspeed[2]
        '''

    #:param alpha, Angle of Attack [rad]
    # return: 
    #Coefficient of Lift, CL_alpha (unitless), Coefficient of Drag,
    #CD_alpha (unitless), Coefficoent of Moment, CM_alpha (unitless)
    def CalculateCoeff_alpha(self, alpha):
        Cd_alpha = Cd_fromA(alpha)  
        Cl_alpha = Cl_fromA(alpha)
        Cm_alpha = VPC.CM0 + VPC.CMalpha * alpha
        #print("returning CdAlpha = " + str(Cd_alpha))
        #print("returning ClAlpha = " + str(Cl_alpha))
        #print("returning CmAlpha = " + str(Cm_alpha))
        return Cl_alpha, Cd_alpha, Cm_alpha


    #you’ll be calculating the first two terms of Beard 4.6 and 4.7, the first three terms
    #of Beard 4.5, and the first four terms of Beard 4.14, 4.15, and 4.16.
    
    def aeroForces(self, state):
        #print("start aeroForces()")
        forcesnMoments = Inputs.forcesMoments()
        if(math.isclose(state.Va, 0)):
            Fl = 0
            Fd = 0
            l = 0
            m = 0
            n = 0
            Fy = 0
        else:
        
           # #print("calculated beta now for actaul function")
            Fl = calcLiftForce(state)
            Fd = calcDragForce(state)
            l = calcMomentL(state)
            m = calcMoment(state)
            n = calcMomentN(state)
            Fy = calcFy(state)

            ##print("end aeroForces")
            forcesnMoments.Fx = Fl
            forcesnMoments.Fy = Fy
            forcesnMoments.Fz = Fd
            forcesnMoments.Mx = l
            forcesnMoments.My = m
            forcesnMoments.Mz = n

            m_1 = [[-1*Fd],[-1*Fl]]
            m_2 = [[math.cos(state.alpha), -1*math.sin(state.alpha)],[math.sin(state.alpha), math.cos(state.alpha)]]
            m_3 = MatrixMath.multiply(m_2, m_1)
            
            forcesnMoments.Fx = m_3[0][0]
            forcesnMoments.Fz = m_3[1][0]
        return forcesnMoments

    def controlForces(self, state, controls):
        forcesnMoments = Inputs.forcesMoments()
        Fprop = self.CalculatePropForces(state.Va, controls.Throttle)
        
        Fl =  1/2* VPC.rho * math.pow(state.Va, 2) * VPC.S * (VPC.CLdeltaE * controls.Elevator)
        Fd =  1/2* VPC.rho * math.pow(state.Va, 2) * VPC.S * (VPC.CDdeltaE * controls.Elevator)
        l = 1/2* VPC.rho * math.pow(state.Va, 2) * VPC.S * VPC.b * (VPC.CldeltaA * controls.Aileron + VPC.CldeltaR * controls.Rudder)
        m = 1/2* VPC.rho * math.pow(state.Va, 2) * VPC.S  * VPC.c * (VPC.CMdeltaE * controls.Elevator)
        n = 1/2* VPC.rho * math.pow(state.Va, 2) * VPC.S * VPC.b * (VPC.CndeltaA * controls.Aileron + VPC.CndeltaR * controls.Rudder)
        Fy = 1/2* VPC.rho * math.pow(state.Va, 2) * VPC.S * (VPC.CYdeltaA * controls.Aileron + VPC.CYdeltaR * controls.Rudder)

        #print("end aeroForces")
        forcesnMoments.Fx = Fd
        forcesnMoments.Fy = Fy
        forcesnMoments.Fz = Fl
        forcesnMoments.Mx = l
        forcesnMoments.My = m
        forcesnMoments.Mz = n

        m_1 = [[-1*Fd],[-1*Fl]]
        m_2 = [[math.cos(state.alpha), -1*math.sin(state.alpha)],[math.sin(state.alpha), math.cos(state.alpha)]]
        m_3 = MatrixMath.multiply(m_2, m_1)
        
        forcesnMoments.Fx = m_3[0][0] 
        forcesnMoments.Fz = m_3[1][0]

        forcesnMoments.Fx += Fprop[0]
        forcesnMoments.Mx += Fprop[1]
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
        Kt = 60/(2*math.pi*VPC.KV)
        a = (VPC.rho * math.pow(VPC.D_prop,5) * VPC.C_Q0)/(4*math.pow(math.pi,2))
        b = (VPC.rho * math.pow(VPC.D_prop,4) * Va * VPC.C_Q1)/(2*math.pi) + (Kt*Kt)/VPC.R_motor
        c = (VPC.rho * math.pow(VPC.D_prop,3) * math.pow(Va, 2) * VPC.C_Q2) - (Kt*((VPC.V_max*Throttle)/VPC.R_motor)) + Kt*VPC.i0
        try:
            omega = (-b + math.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
        except:
   # print("Crashed Propeller Model doing square root (imaginary), Va = {}, dT = {}, b = {}, c = {}".format(Va, DeltaT, b, c))
            omega = 100.0
        #omega = (-1*b + math.sqrt(math.pow(b,2) - (4*a*c)))/(2*a)
        J =  (2* math.pi * Va)/(omega*VPC.D_prop)
        Ct = VPC.C_T0 + VPC.C_T1 * J + VPC.C_T2 * math.pow(J,2)
        Cq = VPC.C_Q0 + VPC.C_Q1 * J + VPC.C_Q2 * math.pow(J,2)
        
        Fprop = (VPC.rho * math.pow(omega, 2) * math.pow(VPC.D_prop, 4) * Ct)/(4*math.pow(math.pi,2))
        Mprop = (-1 * VPC.rho * math.pow(omega, 2) * math.pow(VPC.D_prop, 5) * Cq)/(4*math.pow(math.pi,2))
        
        return (Fprop, Mprop)

    def updateForces(self, state, controls, wind=None):
        if wind is not None:
            #wind.Update(wind.Wu,wind.Wv, wind.Ww)
            airspeed = self.CalculateAirspeed(state, wind)
            state.Va = airspeed[0]
            state.alpha = airspeed[1]
            state.beta = airspeed[2]
        else:
            state.Va = math.hypot(state.u, state.v, state.w)
            state.alpha = math.atan2(state.w,state.u)
            if(math.isclose(state.Va, 0)):
                state.beta = 0
            else:
                state.beta = math.asin(state.v/math.hypot(state.u, state.v, state.w))
        forcesnMoments = Inputs.forcesMoments()
        aF = self.aeroForces(state)
        cF = self.controlForces(state, controls)
        gF = self.gravityForces(state)




        forcesnMoments.Fx = aF.Fx + cF.Fx + gF.Fx
        forcesnMoments.Fy = aF.Fy + cF.Fy + gF.Fy
        forcesnMoments.Fz = aF.Fz + cF.Fz + gF.Fz

        forcesnMoments.Mx = aF.Mx + cF.Mx
        forcesnMoments.My = aF.My + cF.My
        forcesnMoments.Mz = aF.Mz + cF.Mz

        return forcesnMoments

    
    def getWindModel(self):

        return self.myWindModel

    def setWindModel(self,windModel):

        self.myWindModel = windModel

    def CalculateAirspeed(self, state, wind):

        Wind_Steady_Mat = [[wind.Wn], [wind.We], [wind.Wd]]
        Wind_Gust_Mat = [[wind.Wu],[wind.Wv],[wind.Ww]]

        #term1 = MatrixMath.multiply(Rotations.euler2DCM(state.yaw, state.pitch, state.roll), Wind_Steady_Mat)
        #WindCombination_InBodyFrame = MatrixMath.add(term1, Wind_Gust_Mat)
        
        Ws = math.hypot(wind.Wn, wind.We, wind.Wd)
        Xw = math.atan2(wind.We, wind.Wn)
        if math.isclose(Ws, 0):
            Yw = 0
        else:
            Yw = -1 * math.asin(wind.Wd/(math.hypot(wind.Wn, wind.We, wind.Wd)))
        
        Azimuth_Elevation_RMat = [
        [(math.cos(Xw)*math.cos(Yw)), (math.sin(Xw)*math.cos(Yw)), (-math.sin(Yw))],
        [(-math.sin(Xw)), (math.cos(Xw)), 0],
        [(math.cos(Xw)*math.sin(Yw)), (math.sin(Xw)*math.sin(Yw)), (math.cos(Yw))]
        ]
        
        WindInBody = MatrixMath.multiply(Rotations.euler2DCM(state.yaw, state.pitch, state.roll),MatrixMath.add(Wind_Steady_Mat, MatrixMath.multiply(MatrixMath.transpose(Azimuth_Elevation_RMat),Wind_Gust_Mat)))
        
        #va = MatrixMath.subtract([[state.u], [state.v], [state.w]], WindInBody)
        AirspeedVec = [
            [state.u-WindInBody[0][0]],
            [state.v-WindInBody[1][0]],
            [state.w-WindInBody[2][0]]
        ]
        va = math.hypot(AirspeedVec[0][0], AirspeedVec[1][0], AirspeedVec[2][0])
        alpha = math.atan2(AirspeedVec[2][0], AirspeedVec[0][0])
        if math.isclose(va, 0): 
            beta = 0
        else:
            beta = math.asin(AirspeedVec[1][0]/va)
        return [va, alpha, beta]

