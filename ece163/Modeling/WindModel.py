import math
import random
from ..Containers import States
from ..Utilities import MatrixMath
from ..Constants import VehiclePhysicalConstants as VPC
from ..Containers import Inputs

debug = False

class WindModel():

    
    def __init__(self, dT = VPC.dT, Va = VPC.InitialSpeed, drydenParamters=VPC.DrydenNoWind):
        
        self.dT = dT
        self.Va = Va
        self.drydenParameters = drydenParamters

        self.myWindState = States.windState()

        self.Gamma_u = [[0]]
        self.Phi_u = [[0]]
        self.H_u = [[0]]


        self.Gamma_v = [[0,0],[0,0]]
        self.Phi_v = [[0],[0]]
        self.H_v = [[0,0]]

        self.Gamma_w = [[0,0],[0,0]]
        self.Phi_w = [[0],[0]]
        self.H_w = [[0,0]]
    

        self.CreateDrydenTransferFns(self.dT, self.Va, self.drydenParameters)

        self.Xu = [[0]]
        self.Xv = [[0,0]]
        self.Xw = [[0,0]]

    def Update(self, uu = None, uv = None, uw = None):

        if uu is None :
            uu = random.gauss(0 , 1)
        if uv is None :
            uv = random.gauss(0 , 1)
        if uw is None :
            uw = random.gauss(0 , 1)
        #returns nothings stored internally

        #IMPLEMENT STEPS 2-5 FROM DRYDEN CHEAT SHEET
        
        #STEP 2 -> x+ = Φx− + ΓuN : Update “state” using random input
        Xu_plus = MatrixMath.add(MatrixMath.multiply(self.Phi_u, self.Xu), MatrixMath.scalarMultiply(uu,self.Gamma_u))
        Xv_plus = MatrixMath.add(MatrixMath.multiply(self.Phi_v, self.Xv), MatrixMath.scalarMultiply(uv, self.Gamma_v))
        Xw_plus = MatrixMath.add(MatrixMath.multiply(self.Phi_w, self.Xw), MatrixMath.scalarMultiply(uw,self.Gamma_w))


        #STEP 3 -> W[u,v,w] = Hx+ : Generate gusts from state
        #W = [[self.myWindState.Wu], [self.myWindState.Wv], [self.myWindState.Ww]]
        X = [[Xu_plus], [Xv_plus], [Xw_plus]]
        self.myWindState.Wu = MatrixMath.multiply(self.H_u, Xu_plus)[0][0]
        self.myWindState.Wv = MatrixMath.multiply(self.H_v, Xv_plus)[0][0]
        self.myWindState.Ww = MatrixMath.multiply(self.H_w, Xw_plus)[0][0]
    
        #STEP 4 -> x- ↤ x+ : Update previous state
        self.Xu = Xu_plus
        self.Xv = Xv_plus
        self.Xw = Xw_plus
        #STEP 5 At next time step, Goto step (2)
    

    def reset(self):
        pass

    def CreateDrydenTransferFns(self, dT, Va, drydenParameters):
        if(Va <= 0):
            if debug:print("ARITHMETIC ERROR RAISED")
            raise ArithmeticError
        else:
    

       
            #Discrete parameritized time equivalent dryden wind model equations in U axis
            if drydenParameters.Lu != 0:    
                
                uExponentTerm = -1*(Va*dT)/drydenParameters.Lu
                Phi_u = [[math.pow(math.e, uExponentTerm)]]
                if debug:print("Phi_u is:")
                if debug:print(Phi_u)
                Gamma_u = [[drydenParameters.Lu/Va * (1 - math.pow(math.e, uExponentTerm))]]
                if debug:print("Gamma_u is:")
                if debug:print(Gamma_u)
                H_u = [[drydenParameters.sigmau * math.sqrt((2*Va)/(math.pi*drydenParameters.Lu))]]
                if debug:print("H_u is:")
                if debug:print(H_u)
            else:
                Phi_u = [[1]]
                Gamma_u = [[0]]
                H_u = [[1]]

            if drydenParameters.Lv != 0:    
                vExponentTerm = -1*(Va*dT)/drydenParameters.Lv
                #Discrete parameritized time equivalent dryden wind model equations in V axis
                eTermV = math.pow(math.e, vExponentTerm)
                gammaterm2_mat =[
                            [(1+vExponentTerm), -1*(math.pow((Va/drydenParameters.Lv),2))*dT],
                            [(dT), (1-vExponentTerm)]
                        ]
                Phi_v = MatrixMath.scalarMultiply(eTermV, gammaterm2_mat)
                if debug:print("Phi_v is:")
                if debug:print(Phi_v)
                phiterm2_mat =[
                                [dT],
                                [math.pow((drydenParameters.Lv/Va),2)*(math.pow(math.e,(Va/drydenParameters.Lv*dT))-1) - (drydenParameters.Lv/Va*dT)]
                            ]
                Gamma_v = MatrixMath.scalarMultiply(eTermV, phiterm2_mat)
                if debug:print("Gamma_v is:")
                if debug:print(Gamma_v)
                H_v = MatrixMath.scalarMultiply((drydenParameters.sigmav*math.sqrt((3*Va)/(math.pi*drydenParameters.Lv))), [[1, (Va/(math.sqrt(3)*drydenParameters.Lv))]])
                if debug:print("H_v is:")
                if debug:print(H_v)
            else:
                Phi_v = [
                            [1,0],
                            [0,1]
                        ]
                Gamma_v = [[0],[0]]
                H_v = [[1, 1]]

            
            #Discrete parameritized time equivalent dryden wind model equations in W axis
            if drydenParameters.Lw != 0:
                wExponentTerm = -1*(Va*dT)/drydenParameters.Lw
                eTermW = math.pow(math.e, wExponentTerm)
                gammawterm2_mat =[
                            [1+wExponentTerm, -1*(math.pow((Va/drydenParameters.Lw),2))*dT],
                            [dT, 1-wExponentTerm]
                        ]
                        #BRUH KEEP TRACK OF BETTER VARIABLE NAMES 
                        #ERROR WAS HARD TO TRACK BECUASE IT WAS USING A WRONG SIMILARY NAMES VARIABLE
                Phi_w = MatrixMath.scalarMultiply(eTermW, gammawterm2_mat)
                if debug:print("Phi_w is:")
                if debug:print(Phi_w)
                phiwterm2_mat =[
                                [dT],
                                [math.pow((drydenParameters.Lw/Va),2)*(math.pow(math.e,(Va/drydenParameters.Lw*dT))-1) - (drydenParameters.Lw/Va*dT)]
                            ]
                Gamma_w = MatrixMath.scalarMultiply(eTermW, phiwterm2_mat)
                if debug:print("Gamma_w is:")
                if debug:print(Gamma_w)
                H_w = MatrixMath.scalarMultiply((drydenParameters.sigmaw*math.sqrt((3*Va)/(math.pi*drydenParameters.Lw))), [[1, (Va/(math.sqrt(3)*drydenParameters.Lw))]])
                if debug:print("H_w is:")
                if debug:print(H_w)
            else:
                Phi_w = [
                            [1,0],
                            [0,1]
                        ]
                Gamma_w = [[0],[0]]
                H_w = [[1, 1]]

            
            
            
            
        
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

    def setWindModelParameters(self, Wn = 0.0, We = 0.0, Wd = 0.0, drydenParamters = VPC.DrydenNoWind):

        self.myWindState.Wn = Wn
        self.myWindState.We = We
        self.myWindState.Wd = Wd
        self.drydenParameters = drydenParamters
        #returns none