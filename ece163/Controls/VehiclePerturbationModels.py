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
    mytf.Va_trim = 0
    mytf.alpha_trim = 0
    mytf.beta_trim = 0
    mytf.gamma_trim = 0
    mytf.theta_trim = 0
    mytf.phi_trim = 0 
    mytf.a_phi1 = 0
    mytf.a_phi2 = 0
    mytf.a_beta1 = 0
    mytf.a_beta2 = 0
    mytf.a_theta1 = 0
    mytf.a_theta2 = 0
    mytf.a_theta3 = 0
    mytf.a_V1 = (((VPC.rho*(trimState.Va)* VPC.S)/VPC.mass)*(-VPC.CD0 + (VPC.CDalpha*trimState.alpha) - (VPC.CDdeltaE*trimInputs.Elevator))) + (1/VPC.mass * dThrust_dVa(trimState.Va,trimInputs.Throttle, 0.01))
    mytf.a_V2 = 0
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