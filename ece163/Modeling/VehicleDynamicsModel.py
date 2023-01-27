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

    #used by integrate state to forward propogate the DCM rotation Matrix 
    def Rexp(self, dT, state, dot):
        rexp = 0
        NORM = math.hypot(state.p, state.q, state.r)
        sx = MatrixMath.skew(state.p,state.q,state.r)
        IDENTITY =[[1,0,0],[0,1,0],[0,0,1]]
        if NORM < 0.2:
            #different taylor series approximation equations
            sinTermApprox = dT - ((math.pow(dT,3)*math.pow(NORM,2))/6) + ((math.pow(dT,5)*math.pow(NORM,4))/120)
            cosTermApprox = (dT*dT)/2 - ((math.pow(dT,4)*math.pow(NORM,2))/24) + ((math.pow(dT,6)*math.pow(NORM,4))/720)
            rexp = MatrixMath.subtract(IDENTITY,MatrixMath.add(MatrixMath.scalarMultiply(sinTermApprox,sx),MatrixMath.scalarMultiply(cosTermApprox, MatrixMath.multiply(sx,sx))))
        else:
            #stuff from hw1.py
            
            angularMag = NORM
            trigTerm = angularMag * dT
            term1 = MatrixMath.subtract(IDENTITY, MatrixMath.scalarMultiply(math.sin(trigTerm)/angularMag, sx))
            rexp = MatrixMath.add(term1, MatrixMath.scalarMultiply((1-math.cos(trigTerm))/math.pow(angularMag,2),MatrixMath.multiply(sx,sx)))

        return rexp

    def Update(self,forcesnmoments):

        print("start of update")

        print("end of update.")

    def derivative(self,state, forcesnmoments):
        #compute the derivative of the given state with the provided forces n moments then return 
        #an updated state where the derivatives replace what they dervied from i.e pqr => PdotQdotRdot
        #forces and moments contains self.fx-z and self.Mx-z

        dState = States.vehicleState()
        POS = [[state.pn],[state.pe],[state.pd]]
        VELOCITY = [[state.u],[state.v],[state.w]]
        EULERS = [[state.roll], [state.pitch], [state.yaw] ]
        ANGULAR_RATES = [[state.p],[state.q],[state.r]]
        
        FORCES = [[forcesnmoments.Fx],[forcesnmoments.Fy],[forcesnmoments.Fz]]
        MOMENTS = [[forcesnmoments.Mx],[forcesnmoments.My],[forcesnmoments.Mz]]
        
        #derivative of positon
        PosistionDerivative = MatrixMath.multiply(MatrixMath.transpose(state.DCM), VELOCITY)
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
        SKEW_TIMES_VELOCITIES = [
            [state.r * state.v - state.q * state.w],
            [state.p * state.w - state.r * state.u],
            [state.q * state.u - state.p * state.v]
        ]
        VelocityDerivative = MatrixMath.add(SKEW_TIMES_VELOCITIES, MatrixMath.scalarMultiply(1/VPC.mass, FORCES))
        dState.u = [0][0]
        dState.v = [1][0]
        dState.w = [2][0]

        
        #Derivative of angular rates
        PQR_DOT_TERM1 = [
            [VPC.Jzz/VPC.Jdet, 0, VPC.Jxz/VPC.Jdet],
            [0, 1/VPC.Jyy, 0],
            [VPC.Jxz/VPC.Jdet, 0, VPC.Jxx/VPC.Jdet]
        ]
        PQR_DOT_TERM2 = MatrixMath.multiply(MatrixMath.skew(state.p,state.q,state.r),MatrixMath.multiply(VPC.Jbody, ANGULAR_RATES))
        pqrDerivative = MatrixMath.multiply(PQR_DOT_TERM1, MatrixMath.add(PQR_DOT_TERM2, MOMENTS))
        dState.p = pqrDerivative[0][0]
        dState.q = pqrDerivative[1][0]
        dState.r = pqrDerivative[2][0]


        #derivitve of Rotation Matrix
        dState.R = MatrixMath.scalarMultiply(-1, MatrixMath.multiply(MatrixMath.skew(state.p,state.q,state.r),state.R))


        return state

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
        