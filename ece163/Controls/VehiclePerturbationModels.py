import math
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Linearized
from ece163.Utilities import MatrixMath
from ece163.Controls import VehicleTrim


def CreateTransferFunction(trimState, trimInputs):

    mytf = Linearized.transferFunctions()
    mytf.Va_trim = math.hypot(trimState.u, trimState.v, trimState.w)
    mytf.alpha_trim = trimState.alpha
    mytf.theta_trim = trimState.pitch
    #mytf.chi_trim = trimState.chi
    if math.isclose(trimState.Va, 0.0):
        mytf.beta_trim = 0.0
    else:
        mytf.beta_trim = trimState.beta
    mytf.gamma_trim = mytf.theta_trim - mytf.alpha_trim
    
    mytf.phi_trim = trimState.roll
    mytf.a_phi1 = -0.5 * VPC.rho * (trimState.Va ** 2.0) * VPC.S * VPC.b * VPC.Cpp * (VPC.b/(2.0*trimState.Va))
    mytf.a_phi2 = 0.5 * VPC.rho * (trimState.Va ** 2.0) * VPC.S * VPC.b * VPC.CpdeltaA
    mytf.a_beta1 = ((-VPC.rho * trimState.Va * VPC.S)/(2.0*VPC.mass))*VPC.CYbeta
    mytf.a_beta2 = ((VPC.rho * trimState.Va * VPC.S)/(2.0*VPC.mass))*VPC.CYdeltaR

    mytf.a_theta1 = -((VPC.rho * (trimState.Va **2.0) * VPC.c * VPC.S)/(2.0* VPC.Jyy))* VPC.CMq * (VPC.c/(2.0*trimState.Va))
    mytf.a_theta2 = -((VPC.rho * (trimState.Va **2.0) * VPC.c * VPC.S)/(2.0* VPC.Jyy))* VPC.CMalpha
    mytf.a_theta3 = ((VPC.rho * (trimState.Va **2.0) * VPC.c * VPC.S)/(2.0* VPC.Jyy))* VPC.CMdeltaE
    mytf.a_V1 = (((VPC.rho*trimState.Va* VPC.S)/VPC.mass)*(VPC.CD0 + (VPC.CDalpha*trimState.alpha) + (VPC.CDdeltaE*trimInputs.Elevator))) - ((1.0/VPC.mass) * dThrust_dVa(trimState.Va,trimInputs.Throttle))
    mytf.a_V2 = (1.0/VPC.mass) * dThrust_dThrottle(trimState.Va, trimInputs.Throttle)
    mytf.a_V3 = VPC.g0 * math.cos(trimState.pitch - trimState.alpha)

    return mytf

def dThrust_dThrottle(Va, Throttle, epsilon =0.01):
    

    VAeroModel = VehicleAerodynamicsModel.VehicleAerodynamicsModel()
    #partial derivative {N/PWM}
    dt_dDeltaT = 0.0
    fxplus =  VAeroModel.CalculatePropForces(Va, Throttle + epsilon)
    fx = VAeroModel.CalculatePropForces(Va, Throttle)

    dt_dDeltaT = (fxplus[0] - fx[0])/epsilon

    return dt_dDeltaT


def dThrust_dVa(Va, Throttle, epsilon=0.5):
    #partial derivative {N-s/m}
    VAeroModel = VehicleAerodynamicsModel.VehicleAerodynamicsModel()
    dt_dVa = 0.0
    fxplus = VAeroModel.CalculatePropForces(Va + epsilon, Throttle)
    fx = VAeroModel.CalculatePropForces(Va, Throttle)
    dt_dVa = (fxplus[0] - fx[0])/epsilon
    return dt_dVa