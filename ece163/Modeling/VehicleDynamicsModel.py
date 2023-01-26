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

    def Rexp(self, dT, state, dot):
        rexp = 0
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
        
        PosistionDerivative = MatrixMath.multiply(MatrixMath.transpose(state.DCM), VELOCITY)
        dState.pn = PosistionDerivative[0][0]
        dState.pe = PosistionDerivative[1][0]
        dState.pd = PosistionDerivative[2][0]


        MATRIX_FOR_EULER_DERIVATIVES = [
                    [1, math.sin(state.roll)*math.tan(state.pitch), math.cos(state.roll)*math.tan(state.pitch)],
                    [0, math.cos(state.roll), -1*math.sin(state.roll)],
                    [0, math.sin(state.roll)/math.cos(state.pitch), math.cos(state.roll)/math.cos(state.pitch)]
        ]
        EulerDerivative = MatrixMath.multiply(MATRIX_FOR_EULER_DERIVATIVES, ANGULAR_RATES)
        dState.yaw = [0][0]
        dState.pitch = [1][0]
        dState.roll = [2][0]


        SKEW_TIMES_VELOCITIES = [
            [state.r * state.v - state.q * state.w],
            [state.p * state.w - state.r * state.u],
            [state.q * state.u - state.p * state.v]
        ]
        VelocityDerivative = MatrixMath.add(SKEW_TIMES_VELOCITIES, MatrixMath.scalarMultiply(1/VPC.mass, FORCES))
        dState.u = [0][0]
        dState.v = [1][0]
        dState.w = [2][0]

        dState.p
        dState.q
        dState.r 


        return state

    def getVehicleDerivative(self):

        return self.state

    def getVehicleState(self):

        return self.state
    
    def reset(self):

        newState = States.vehicleState()
        self.state = newState
    
    def setVehicleDot(self, dot):

        dot = self.dot

    def setVehicleState(self, state):

        state = self.state
        