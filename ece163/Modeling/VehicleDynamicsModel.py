import math
from ..Containers import States
from ..Utilities import MatrixMath as mm
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC

class VehicleDynamicsModel():

    def __init__(self, dT=VPC.dT):
        self.dot = States.vehicleState()
        self.state = States.vehicleState()
        self.dT = dT

    def setVehicleState(self, state):
        self.state = state
        return

    def getVehicleState(self):
        return self.state

    def setVehicleDerivative(self, dot):
        self.dot = dot
        return

    def getVehicleDerivative(self):
        return self.dot

    def reset(self):
        self.setVehicleState(States.vehicleState())
        self.setVehicleDerivative(States.vehicleState())
        return

    def Update(self, forcesMoments):
        state = self.getVehicleState()
        dot = self.derivative(state, forcesMoments)
        self.state = self.IntegrateState(self.dT, state, dot)
        return

    def Rexp(self, dT, state, dot):

        p = state.p + (dot.p * (dT/2.0))
        q = state.q + (dot.q * (dT/2.0))
        r = state.r + (dot.r * (dT/2.0))
        w = math.hypot(p, q, r)
        skew_1 = mm.skew(p, q, r)
        skew_2 = mm.multiply(skew_1, skew_1)

        I_matrix = [[1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0]]

        if w >= 0.2:
            divide_1 = (math.sin(w * dT)) / w
            matrix_1 = mm.scalarMultiply(divide_1, skew_1)
            divide_2 = (1.0 - math.cos(w * dT)) / (w * w)
            matrix_2 = mm.scalarMultiply(divide_2, skew_2)

            half_matrix = mm.subtract(I_matrix, matrix_1)
            full_matrix = mm.add(half_matrix, matrix_2)

        else:

            divide_1 = dT - ((dT ** 3.0) * (w ** 2.0) / 6.0) + ((dT ** 5.0) * (w ** 4.0) / 120.0)
            matrix_1 = mm.scalarMultiply(divide_1, skew_1)
            divide_2 = (dT ** 2.0) / 2.0 - ((dT ** 4.0) * (w ** 2.0) / 24.0) + ((dT ** 6.0) * (w ** 4.0) / 720.0)
            matrix_2 = mm.scalarMultiply(divide_2, skew_2)

            half_matrix = mm.subtract(I_matrix, matrix_1)
            full_matrix = mm.add(half_matrix, matrix_2)

        return full_matrix

    def derivative(self, state, forcesMoments):
        dot = States.vehicleState()

        p = state.p
        q = state.q
        r = state.r

        yaw = state.yaw
        pitch = state.pitch
        roll = state.roll

        u = state.u
        v = state.v
        w = state.w

        skew_m = mm.skew(p, q, r)
        skew_negate = mm.scalarMultiply(-1.0, skew_m)
        skew_final = mm.multiply(skew_negate, [[u], [v], [w]])
        state.R = Rotations.euler2DCM(yaw, pitch, roll)
        forces_mass = mm.scalarMultiply(1.0 / VPC.mass, [[forcesMoments.Fx], [forcesMoments.Fy], [forcesMoments.Fz]])
        final_uvw = mm.add(forces_mass, skew_final)

        ypr_matrix = [[1.0, math.sin(roll) * math.tan(pitch), math.cos(roll) * math.tan(pitch)],
                      [0.0, math.cos(roll), -math.sin(roll)],
                      [0.0, (math.sin(roll)) / (math.cos(pitch)), (math.cos(roll)) / (math.cos(pitch))]]
        deriv_ypr = mm.multiply(ypr_matrix, [[p], [q], [r]])

        j_omega = mm.multiply(VPC.Jbody, [[p], [q], [r]])
        skew_jw = mm.multiply(skew_negate, j_omega)
        dpqr_matrix = mm.add(skew_jw, [[forcesMoments.Mx], [forcesMoments.My], [forcesMoments.Mz]])
        final_dpqr = mm.multiply(VPC.JinvBody, dpqr_matrix)

        R_deriv = mm.multiply(skew_negate, state.R)

        ned_deriv = mm.multiply(mm.transpose(state.R), [[u], [v], [w]])

        chi = math.atan2(ned_deriv[1][0], ned_deriv[2][0])

        dot.pn = ned_deriv[0][0]
        dot.pe = ned_deriv[1][0]
        dot.pd = ned_deriv[2][0]
        dot.u = final_uvw[0][0]
        dot.v = final_uvw[1][0]
        dot.w = final_uvw[2][0]
        dot.yaw = deriv_ypr[2][0]
        dot.pitch = deriv_ypr[1][0]
        dot.roll = deriv_ypr[0][0]
        dot.p = final_dpqr[0][0]
        dot.q = final_dpqr[1][0]
        dot.r = final_dpqr[2][0]
        dot.dcm = R_deriv
        dot.chi = chi

        return dot

    def ForwardEuler(self, dT, state, dot):
        n = state.pn + dot.pn * dT
        e = state.pe + dot.pe * dT
        d = state.pd + dot.pd * dT

        u = state.u + dot.u * dT
        v = state.v + dot.v * dT
        w = state.w + dot.w * dT

        p = state.p + dot.p * dT
        q = state.q + dot.q * dT
        r = state.r + dot.r * dT

        return States.vehicleState(pn=n, pe=e, pd=d, u=u, v=v, w=w, p=p, q=q, r=r)

    def IntegrateState(self, dT, state, dot):

        p = self.ForwardEuler(dT, state, dot).p
        q = self.ForwardEuler(dT, state, dot).q
        r = self.ForwardEuler(dT, state, dot).r

        n = self.ForwardEuler(dT, state, dot).pn
        e = self.ForwardEuler(dT, state, dot).pe
        d = self.ForwardEuler(dT, state, dot).pd

        u = self.ForwardEuler(dT, state, dot).u
        v = self.ForwardEuler(dT, state, dot).v
        w = self.ForwardEuler(dT, state, dot).w

        R_k1 = mm.multiply(self.Rexp(dT, state, dot), state.R)

        _vState = States.vehicleState(pn=n, pe=e, pd=d,
                                   u=u, v=v, w=w,
                                   p=p, q=q, r=r, dcm=R_k1)

        _vState.alpha = state.alpha
        _vState.beta = state.beta
        _vState.Va = state.Va
        _vState.chi = math.atan2(dot.pe, dot.pn)

        return _vState
"""
class VehicleDynamicsModel:
    
    def __init__(self, dT = VPC.dT):
        self.state = States.vehicleState()
        self.dot = States.vehicleState()
        self.dT = dT
        return 

    def ForwardEuler(self, dT, state, dot):
        new_state = States.vehicleState()
        n_1 = state.pn + dot.pn*dT
        e_1 = state.pe + dot.pe*dT
        d_1 = state.pd + dot.pd*dT

        u_1 = state.u + dot.u * dT
        v_1 = state.v + dot.v * dT
        w_1 = state.w + dot.w * dT

        p_1 = state.p + dot.p*dT
        q_1 = state.q + dot.q*dT
        r_1 = state.r + dot.r*dT


        new_state.pn=n_1
        new_state.pe=e_1
        new_state.pd=d_1
        new_state.u=u_1
        new_state.v=v_1
        new_state.w=w_1
        new_state.p=p_1
        new_state.q=q_1
        new_state.r=r_1
        return new_state
    
    def IntegrateState(self, dT, state, dot):
        #calculate forward integration for pqr,PnPePd, uvw
        fE = self.ForwardEuler(dT,state,dot)
        Rexp = self.Rexp(dT,state, dot)
        fE.R = MatrixMath.multiply(Rexp, state.R)

        
        fE.yaw, fE.pitch, fE.roll = Rotations.dcm2Euler(fE.R)
        fE.alpha = state.alpha
        fE.beta = state.beta
        fE.Va = state.Va
        fE.chi = math.atan2 ( dot.pe , dot.pn )
        return fE

    #used by integrate state to forward propogate the DCM rotation Matrix 
    def Rexp(self, dT, state, dot):
    
        p_0 = state.p + (dot.p*(dT/2.0))
        q_0 = state.q + (dot.q*(dT/2.0))
        r_0 = state.r + (dot.r*(dT/2.0))
        skew_matrix = MatrixMath.skew(p_0, q_0, r_0)
        skew_matrix_2 = MatrixMath.multiply(skew_matrix, skew_matrix)
        identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        omega = math.hypot(p_0, q_0, r_0)
        if omega < 0.2:
            sin_part = (dT) - (((dT**3.0)*(omega**2.0))/6.0) + (((dT**5.0)*(omega**4.0))/120.0)
            cos_part = ((dT**2.0)/2.0) - (((dT**4)*(omega**2.0))/24.0) + (((dT**6)*(omega**4.0))/720.0)
            sin_matrix = MatrixMath.scalarMultiply(sin_part, skew_matrix)
            cos_matrix = MatrixMath.scalarMultiply(cos_part, skew_matrix_2)
            temp = MatrixMath.subtract(identity, sin_matrix)
            rexp = MatrixMath.add(temp, cos_matrix)
        else:
            sin_part = math.sin(omega*dT)/omega
            cos_part = (1.0 - math.cos(omega*dT))/(omega**2.0)
            sin_matrix = MatrixMath.scalarMultiply(sin_part, skew_matrix)
            cos_matrix = MatrixMath.scalarMultiply(cos_part, skew_matrix_2)
            temp = MatrixMath.subtract(identity, sin_matrix)
            rexp = MatrixMath.add(temp, cos_matrix)
        return rexp

    def Update(self,forcesnmoments):

        #print("start of update")
        
    
        self.dot = self.derivative(self.state,forcesnmoments)
        self.state = self.IntegrateState(self.dT, self.state, self.dot)
    
        return
        #print("end of update.")

    def derivative(self,state, forcesnmoments):
        #compute the derivative of the given state with the provided forces n moments then return 
        #an updated state where IntegrateStatethe derivatives replace what they dervied from i.e pqr => PdotQdotRdot
        #forces and moments contains self.fx-z and self.Mx-z
        
        newState = States.vehicleState()
        p_0 = state.p
        q_0 = state.q
        r_0 = state.r
        pitch_0 = state.pitch
        roll_0 = state.roll
        skew_matrix = MatrixMath.skew(p_0, q_0, r_0)
        neg_skew = MatrixMath.scalarMultiply(-1.0, skew_matrix)
        R_ddt = MatrixMath.multiply(neg_skew, state.R) # Attitude cheatsheet (19)
        #----------------------------------# Attitude cheatsheet (26)
        mmatrix = [[1.0, math.sin(roll_0)*math.tan(pitch_0), math.cos(roll_0)*math.tan(pitch_0)],
                   [0.0, math.cos(roll_0), -1.0*math.sin(roll_0)],
                   [0.0, (math.sin(roll_0)/math.cos(pitch_0)),( math.cos(roll_0)/math.cos(pitch_0))]]
        ypr_ddt = MatrixMath.multiply(mmatrix, [[state.p], [state.q], [state.r]])
        #-----------------------------------Lecture Equations of motion 1.3
        m_force = MatrixMath.scalarMultiply(1.0/VPC.mass, [[forcesnmoments.Fx], [forcesnmoments.Fy], [forcesnmoments.Fz]])
        scew_uvw = MatrixMath.multiply(neg_skew, [[state.u], [state.v], [state.w]])
        uvw_ddt = MatrixMath.add(m_force, scew_uvw)
        #-------------------------------------Lecture Equations of motion 1.3
        R_trans = MatrixMath.transpose(state.R)
        pned_ddt = MatrixMath.multiply(R_trans, [[state.u], [state.v], [state.w]])
        #-------------------------------------Lecture Equations of motion 1.3
        J_w = MatrixMath.multiply(VPC.Jbody, [[state.p], [state.q], [state.r]])
        J_skew = MatrixMath.multiply(neg_skew, J_w)
        temp = MatrixMath.add(J_skew, [[forcesnmoments.Mx], [forcesnmoments.My], [forcesnmoments.Mz]])
        pqr_ddt = MatrixMath.multiply(VPC.JinvBody, temp)
        #---------------------------------------
        '''
        roll_0 = ypr_ddt[2][0]
        pitch_0 = ypr_ddt[1][0]

        R_matrix = [[1.0, math.sin(roll_0) * math.tan(pitch_0), math.cos(roll_0) * math.tan(pitch_0)],
                   [0.0, math.cos(roll_0), -1.0 * math.sin(roll_0)],
                   [0.0, (math.sin(roll_0) / math.cos(pitch_0)), (math.cos(roll_0) / math.cos(pitch_0))]]
        print("Rotational_matrix, rolll:", roll_0, "pitch:", pitch_0)
        mm.matrixPrint(R_ddt)

        #R_matrix = Rotations.euler2DCM(ypr_ddt[0][0], ypr_ddt[1][0], ypr_ddt[2][0])
        print("euler to dcm matrix")
        mm.matrixPrint(R_matrix)
        '''
        #---------------------------------------
        chi_ddt = math.atan2(pned_ddt[1][0], pned_ddt[2][0])
        newState.pn=pned_ddt[0][0]
        newState.pe = pned_ddt[1][0]
        newState.pd=pned_ddt[2][0]
        newState.u=uvw_ddt[0][0]
        newState.v=uvw_ddt[1][0]
        newState.w = uvw_ddt[2][0]
        newState.roll = ypr_ddt[0][0]
        newState.pitch=ypr_ddt[1][0]
        newState.yaw = ypr_ddt[2][0]
        newState.p = pqr_ddt[0][0]
        newState.q=pqr_ddt[1][0]
        newState.r=pqr_ddt[2][0]
        #newState.dcm=R_ddt
        newState.R = R_ddt
        newState.chi = chi_ddt
        return newState
        
        
      
        # Function to compute the time-derivative of the state given body frame forces and moments
        # All formulas are from lecture/book
        
    

    def getVehicleDerivative(self):

        return self.dot

    def setVehicleDerivative(self, dot):

        self.dot = dot
        return 

    def getVehicleState(self):

        return self.state
    
    def reset(self):

        state = States.vehicleState()
        self.setVehicleState(state)
        self.setVehicleDerivative(state)
        return
    
    def setVehicleDot(self, dot):

        self.dot = dot
        return

    def setVehicleState(self, state):

        self.state = state
        return
        
"""