import math
import random
from ..Containers import States
from ..Utilities import MatrixMath
from ..Constants import VehiclePhysicalConstants as VPC


class WindModel:

    def __init__(self, dT = VPC.dT, Va = VPC.InitialSpeed, drydenParamters=VPC.DrydenNoWind):
        
        self.dT = dT
        self.Va = Va
        self.drydenParameters = drydenParamters

        self.myWindState = States.windState()

        self.Gamma_u = [[0]]
        self.Phi_u = [[0]]
        self.H_u = [[0]]


        self.Gamma_v = [[0,0], [0,0]]
        self.Phi_v =[[0], [0]]
        self.H_v = [[0,0]]

        self.Gamma_w = [[0,0],[0,0]]
        self.Phi_w = [[0],[0]]
        self.H_w = [[0,0]]

        #self.Gamma = [[self.Gamma_u], [self.Gamma_v], [self.Gamma_w]]
        #self.Phi = [[self.Phi_u], [self.Phi_v], [self.Phi_w]]
        #self.H = [[self.H_u], [self.H_v], [self.H_w]]


    def Update(self, uu = None, uv = None, uw = None):

        pass
        #returns nothings stored internally

    def reset(self):
        pass

    def CreateDrydenTransferFns(dT, Va, drydenParameters):
        pass
        #returns none

    def getDrydenTransferFns(self):
        return self.Phi_u , self.Gamma_u , self.H_u , self.Phi_v , self.Gamma_v , self.H_v , self.Phi_w , self.Gamma_w , self.H_w

    def getWind(self):

        return self.myWindState

    def setWind(self, windState):

        self.myWindState = windState

    def setWindModelParameters(self, Wn = 0.0, We = 0.0, Wd = 0.0, drydenParameters = VPC.DrydenNoWind):

        self.myWindState.Wn = Wn
        self.myWindState.We = We
        self.myWindState.Wd = Wd
        self.drydenParameters = drydenParameters
        #returns none