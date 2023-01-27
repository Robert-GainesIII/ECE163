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
        newState.pn = state.pn + dot.pn*dT
        newState.pe = state.pe + dot.pe*dT
        newState.pd = state.pd + dot.pd*dT

        newState.p = state.p + dot.p*dT
        newState.q = state.q + dot.q*dT
        newState.r = state.r + dot.r*dT

        newState.u = state.u + dot.u*dT
        newState.v = state.v + dot.v*dT
        newState.w = state.w + dot.w*dT
        return newState
    
    def IntegrateState(self, dT, state, dot):
        #calculate forward integration for pqr,PnPePd, uvw
        Rexp = self.Rexp(dT,state, dot)
        Rnext = MatrixMath.multiply(Rexp, state.R)

        fE = self.ForwardEuler(dT,state,dot)

        newState = States.vehicleState(pn=fE.pn, pe=fE.pe, pd=fE.pd, u=fE.u, v=fE.v, w=fE.w, p=fE.p, q=fE.q, r=fE.r, dcm=Rnext)
        newState.alpha = state.alpha
        newState.beta = state.beta
        newState.Va = state.Va
        newState.chi = math.atan2 ( dot.pe , dot.pn )
        return newState

    #used by integrate state to forward propogate the DCM rotation Matrix 
    def Rexp(self, dT, state, dot):
        p = state.p + (dot.p * dT/2)
        q = state.q + (dot.q * dT/2)
        r = state.r + (dot.r * dT/2)


        rexp = 0
        NORM = math.hypot(p, q, r)
        sx = MatrixMath.skew(p,q,r)
        IDENTITY =[[1,0,0],[0,1,0],[0,0,1]]
        if NORM < 0.2:
            #different taylor series approximation equations
            sinTermApprox = dT - ((math.pow(dT,3)*math.pow(NORM,2))/6) + ((math.pow(dT,5)*math.pow(NORM,4))/120)
            cosTermApprox = (dT*dT)/2 - ((math.pow(dT,4)*math.pow(NORM,2))/24) + ((math.pow(dT,6)*math.pow(NORM,4))/720)
            rexp = MatrixMath.add(MatrixMath.subtract(IDENTITY,MatrixMath.scalarMultiply(sinTermApprox,sx)), MatrixMath.scalarMultiply(cosTermApprox, MatrixMath.multiply(sx,sx)))
        else:
            #stuff from hw1.py
            
            angularMag = NORM
            trigTerm = angularMag * dT
            term1 = MatrixMath.subtract(IDENTITY, MatrixMath.scalarMultiply(math.sin(trigTerm)/angularMag, sx))
            rexp = MatrixMath.add(term1, MatrixMath.scalarMultiply((1-math.cos(trigTerm))/math.pow(angularMag,2),MatrixMath.multiply(sx,sx)))

        return rexp

    def Update(self,forcesnmoments):

        #print("start of update")
        
        state = self.getVehicleState()
        derivative = self.derivative(state,forcesnmoments)
        integrated = self.IntegrateState(self.dT, state, derivative)
        self.setVehicleState(integrated)
        #print("end of update.")

    def derivative(self,state, forcesnmoments):
        #compute the derivative of the given state with the provided forces n moments then return 
        #an updated state where IntegrateStatethe derivatives replace what they dervied from i.e pqr => PdotQdotRdot
        #forces and moments contains self.fx-z and self.Mx-z

        dState = States.vehicleState()
        POS = [[state.pn],[state.pe],[state.pd]]
        VELOCITY = [[state.u],[state.v],[state.w]]
        EULERS = [[state.roll], [state.pitch], [state.yaw] ]
        ANGULAR_RATES = [[state.p],[state.q],[state.r]]
        SKEW = MatrixMath.skew(state.p,state.q,state.r)
        
        FORCES = [[forcesnmoments.Fx],[forcesnmoments.Fy],[forcesnmoments.Fz]]
        MOMENTS = [[forcesnmoments.Mx],[forcesnmoments.My],[forcesnmoments.Mz]]
        
        #derivative of positon
        PosistionDerivative = MatrixMath.multiply(MatrixMath.transpose(state.R), VELOCITY)
        dState.pn = PosistionDerivative[0][0]
        dState.pe = PosistionDerivative[1][0]
        dState.pd = PosistionDerivative[2][0]

        #derivative of euler angles
        MATRIX_FOR_EULER_DERIVATIVES = [
                    [1, math.sin(state.roll)*math.tan(state.pitch), math.cos(state.roll)*math.tan(state.pitch)],
                    [0, math.cos(state.roll), -1*math.sin(state.roll)],
                    [0, math.sin(state.roll)/math.cos(state.pitch), math.cos(state.roll)/math.cos(state.pitch)]
        ]
        EulerDerivative = MatrixMath.multiply(MATRIX_FOR_EULER_DERIVATIVES, ANGULAR_RATES)
        dState.yaw = [0][0]
        dState.pitch = [1][0]
        dState.roll = [2][0]


        #Deriative of velocity
        NEGSKEW = MatrixMath.scalarMultiply(-1, SKEW)
        VelocityDerivative = MatrixMath.add(MatrixMath.multiply(NEGSKEW,VELOCITY), MatrixMath.scalarMultiply(1/VPC.mass, FORCES))
        dState.u = VelocityDerivative[0][0]
        dState.v = VelocityDerivative[1][0]
        dState.w = VelocityDerivative[2][0]

        
        #Derivative of angular rates
        PQR_DOT_TERM1 = [
            [VPC.Jzz/VPC.Jdet, 0, VPC.Jxz/VPC.Jdet],
            [0, 1/VPC.Jyy, 0],
            [VPC.Jxz/VPC.Jdet, 0, VPC.Jxx/VPC.Jdet]
        ]
        #jinverse multplied by -skew*j*pqr + moments
        pqrDerivative = MatrixMath.multiply(VPC.JinvBody, MatrixMath.add(MatrixMath.multiply(MatrixMath.multiply(MatrixMath.scalarMultiply(-1,SKEW),VPC.Jbody),ANGULAR_RATES), MOMENTS))
        dState.p = pqrDerivative[0][0]
        dState.q = pqrDerivative[1][0]
        dState.r = pqrDerivative[2][0]


        #derivitve of Rotation Matrix
        dState.R = MatrixMath.scalarMultiply(-1, MatrixMath.multiply(MatrixMath.skew(state.p,state.q,state.r),state.R))


        return dState

    def getVehicleDerivative(self):

        return self.dot

    def getVehicleState(self):

        return self.state
    
    def reset(self):

        newState = States.vehicleState()
        self.state = newState
    
    def setVehicleDot(self, dot):

        self.dot = dot

    def setVehicleState(self, state):

        self.state = state
        