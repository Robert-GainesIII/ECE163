import math
import random
from ..Containers import States
from ..Utilities import MatrixMath
from ..Constants import VehiclePhysicalConstants as VPC
from ..Containers import Inputs


class WindModel(dT = 0.01, Va = 25.0, drydenParamters = Inputs.drydenParameters(Lu=0.0, Lv=0.0, Lw = 0.0, sigmau = 0.0, sigmav = 0.0, sigmaw=0.0)):

    def __init__(self, dT = VPC.dT, Va = VPC.InitialSpeed, drydenParamters=VPC.DrydenNoWind):
        
        self.dT = dT
        self.Va = Va
        self.drydenParameters = drydenParamters

        self.myWindState = States.windState()

        self.Gamma_u = 0
        self.Phi_u = 0
        self.H_u = 0


        self.Gamma_v = 0
        self.Phi_v = 0
        self.H_v = 0

        self.Gamma_w = 0
        self.Phi_w = 0
        self.H_w = 0

        self.CreateDrydenTransferFns(self.dT, self.Va, self.drydenParameters)

        self.Xu = [[self.Gamma_u], [self.Gamma_v], [self.Gamma_w]]
        self.Xv = [[self.Phi_u], [self.Phi_v], [self.Phi_w]]
        self.Xw = [[self.H_u], [self.H_v], [self.H_w]]


    def Update(self, uu = None, uv = None, uw = None):

        pass
        #returns nothings stored internally

    def reset(self):
        pass

    def CreateDrydenTransferFns(self, dT, Va, drydenParameters):
        uExponentTerm = -(Va/drydenParameters.Lu)*dT
        vExponentTerm = -(Va/drydenParameters.Lv)*dT
        wExponentTerm = -(Va/drydenParameters.Lw)*dT
        
        #Discrete parameritized time equivalent dryden wind model equations in U axis
        Gamma_u = [[math.pow(math.e, uExponentTerm)]]
        Phi_u = drydenParameters.Lu/Va * (1 - math.pow(math.e, uExponentTerm))
        H_u = drydenParameters.sigmau * math.sqrt((2*Va)/(math.pi*drydenParameters.Lu))

        #Discrete parameritized time equivalent dryden wind model equations in V axis
        eTermV = math.pow(math.e, vExponentTerm)
        gammaterm2_mat =[
                    [(1+vExponentTerm), -1*(math.pow((Va/drydenParameters.Lv),2))*dT],
                    [(dT), (1-vExponentTerm)]
                ]
        Gamma_v = MatrixMath.scalarMultiply(eTermV, gammaterm2_mat)
        phiterm2_mat =[
                        [dT],
                        [math.pow((drydenParameters.Lv/Va),2)*(math.pow(math.e,(Va/drydenParameters.Lv*dT))-1) - (drydenParameters.Lv/Va*dT)]
                    ]
        Phi_v = MatrixMath.scalarMultiply(eTermV, phiterm2_mat)
        H_v = MatrixMath.scalarMultiply((drydenParameters.sigmav*math.sqrt((3*Va)/(math.pi*drydenParameters.Lv))), [[1, (Va/(math.sqrt(3)*drydenParameters.Lv))]])

        #Discrete parameritized time equivalent dryden wind model equations in W axis
        eTermW = math.pow(math.e, wExponentTerm)
        gammawterm2_mat =[
                    [(1+wExponentTerm), -1*(math.pow((Va/drydenParameters.Lw),2))*dT],
                    [(dT), (1-wExponentTerm)]
                ]
        Gamma_w = MatrixMath.scalarMultiply(eTermW, gammaterm2_mat)
        phiwterm2_mat =[
                        [dT],
                        [math.pow((drydenParameters.Lw/Va),2)*(math.pow(math.e,(Va/drydenParameters.Lw*dT))-1) - (drydenParameters.Lw/Va*dT)]
                    ]
        Phi_w = MatrixMath.scalarMultiply(eTermW, phiterm2_mat)
        H_w = MatrixMath.scalarMultiply((drydenParameters.sigmaw*math.sqrt((3*Va)/(math.pi*drydenParameters.Lw))), [[1, (Va/(math.sqrt(3)*drydenParameters.Lw))]])

        
        self.Gamma_u = Gamma_u
        self.Phi_u = Phi_u
        self.H_u = H_u


        self.Gamma_v = Gamma_v
        self.Phi_v = Phi_v
        self.H_v = H_v

        self.Gamma_w = Gamma_w
        self.Phi_w = Phi_w
        self.H_w = H_w



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