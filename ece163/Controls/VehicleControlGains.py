import math
import pickle
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Controls
from ece163.Containers import Linearized
from ece163.Utilities import MatrixMath
from ece163.Utilities import Rotations



def computeGains(tuningParameters, linearizedModel):
    controlGains = Controls.controlGains()
    # Lateral Gains
    controlGains.kp_roll = tuningParameters.Wn_roll ** 2 / linearizedModel.a_phi2
    controlGains.kd_roll = ((2* tuningParameters.Zeta_roll * tuningParameters.Wn_roll) - linearizedModel.a_phi1)/linearizedModel.a_phi2
    controlGains.ki_roll = 0.001
    controlGains.kp_sideslip = ((2 * tuningParameters.Zeta_sideslip * tuningParameters.Wn_sideslip)-linearizedModel.a_beta1)/linearizedModel.a_beta2

    controlGains.ki_sideslip = 1/linearizedModel.a_beta2 * ((linearizedModel.a_beta1 + linearizedModel.a_beta2*controlGains.kp_sideslip)/(2*tuningParameters.Zeta_sideslip))**2
    controlGains.kp_course = (2 * tuningParameters.Zeta_course * tuningParameters.Wn_course * linearizedModel.Va_trim)/ VPC.g0 #add ground speed!
    controlGains.ki_course = (tuningParameters.Wn_course **2 * linearizedModel.Va_trim)/VPC.g0 # add ground speed!
    # Longitudinal Gains
    controlGains.kp_pitch = (tuningParameters.Wn_pitch **2 - linearizedModel.a_theta2)/linearizedModel.a_theta3
    controlGains.kd_pitch = (2* tuningParameters.Zeta_pitch * tuningParameters.Wn_pitch - linearizedModel.a_theta1)/linearizedModel.a_theta3
    kDC = (controlGains.kp_pitch * linearizedModel.a_theta3)/(linearizedModel.a_theta2 + controlGains.kp_pitch * linearizedModel.a_theta3)
    controlGains.kp_altitude = (2* tuningParameters.Zeta_altitude * tuningParameters.Wn_altitude)/(kDC * linearizedModel.Va_trim)
    controlGains.ki_altitude = tuningParameters.Wn_altitude **2 / (kDC * linearizedModel.Va_trim)
    controlGains.kp_SpeedfromThrottle = (2* tuningParameters.Zeta_SpeedfromThrottle * tuningParameters.Wn_SpeedfromThrottle - linearizedModel.a_V1)/linearizedModel.a_V2
    controlGains.ki_SpeedfromThrottle = tuningParameters.Wn_SpeedfromThrottle ** 2 / linearizedModel.a_V2
    controlGains.kp_SpeedfromElevator = (linearizedModel.a_V1 - 2*tuningParameters.Zeta_SpeedfromElevator * tuningParameters.Wn_SpeedfromElevator)/(kDC*VPC.g0)
    controlGains.ki_SpeedfromElevator = -(tuningParameters.Wn_SpeedfromElevator ** 2)/(kDC * VPC.g0)
    return controlGains

def computeTuningParameters(controlGains, linearizedModel):

    tuningParams = Controls.controlTuning()
    
    """
    controlGains.kp_roll = tuningParameters.Wn_roll ** 2 / linearizedModel.a_phi2
    controlGains.kd_roll = ((2* tuningParameters.Zeta_roll * tuningParameters.Wn_roll) - linearizedModel.a_phi1)/linearizedModel.a_phi2
    controlGains.ki_roll = 0.001
    
    controlGains.kp_course = (2 * tuningParameters.Zeta_course * tuningParameters.Wn_course * linearizedModel.Va_trim)/ VPC.g0 #add ground speed!
    controlGains.ki_course = (tuningParameters.Wn_course **2 * linearizedModel.Va_trim)/VPC.g0 # add ground speed!
    # Longitudinal Gains
    controlGains.kp_pitch = (tuningParameters.Wn_pitch **2 - linearizedModel.a_theta2)/linearizedModel.a_theta3
    controlGains.kd_pitch = (2* tuningParameters.Zeta_pitch * tuningParameters.Wn_pitch - linearizedModel.a_theta1)/linearizedModel.a_theta3
    kDC = (controlGains.kp_pitch * linearizedModel.a_theta3)/(linearizedModel.a_theta2 + controlGains.kp_pitch * linearizedModel.a_theta3)
    controlGains.kp_altitude = (2* tuningParameters.Zeta_altitude * tuningParameters.Wn_altitude)/(kDC * linearizedModel.Va_trim)
    controlGains.ki_altitude = tuningParameters.Wn_altitude **2 / (kDC * linearizedModel.Va_trim)
    controlGains.kp_SpeedfromThrottle = (2* tuningParameters.Zeta_SpeedfromThrottle * tuningParameters.Wn_SpeedfromThrottle - linearizedModel.a_V1)/linearizedModel.a_V2
    controlGains.ki_SpeedfromThrottle = tuningParameters.Wn_SpeedfromThrottle ** 2 / linearizedModel.a_V2
    controlGains.kp_SpeedfromElevator = (linearizedModel.a_V1 - 2*tuningParameters.Zeta_SpeedfromElevator * tuningParameters.Wn_SpeedfromElevator)/(kDC*VPC.g0)
    controlGains.ki_SpeedfromElevator = -(tuningParameters.Wn_SpeedfromElevator ** 2)/(kDC * VPC.g0)
    """
    
    tuningParams.Wn_roll  = math.sqrt(controlGains.kp_roll * linearizedModel.a_phi2)
    tuningParams.Zeta_roll  = (controlGains.kd_roll * linearizedModel.a_phi2 + linearizedModel.a_phi1)/ (tuningParams.Wn_roll*2)

    tuningParams.Wn_course  = math.sqrt((controlGains.ki_course * VPC.g0)/linearizedModel.Va_trim)
    tuningParams.Zeta_course  = (controlGains.kp_course *  VPC.g0)/(tuningParams.Wn_course * linearizedModel.Va_trim* 2)
    
   
    tuningParams.Zeta_sideslip  = (linearizedModel.a_beta1 + linearizedModel.a_beta2*controlGains.kp_sideslip)/(math.sqrt(controlGains.ki_sideslip * linearizedModel.a_beta2)* 2)
    tuningParams.Wn_sideslip  = (controlGains.kp_sideslip * linearizedModel.a_beta2 + linearizedModel.a_beta1)/(2 * tuningParams.Zeta_sideslip )
    #tuning knobs for longitudinal control
   
    tuningParams.Wn_pitch  = math.sqrt(controlGains.kp_pitch * linearizedModel.a_theta3 + linearizedModel.a_theta2)
    tuningParams.Zeta_pitch  = (controlGains.kd_pitch * linearizedModel.a_theta3  + linearizedModel.a_theta1)/(tuningParams.Wn_pitch * 2)

 
    kDC = (controlGains.kp_pitch * linearizedModel.a_theta3)/(linearizedModel.a_theta2 + controlGains.kp_pitch * linearizedModel.a_theta3)

    tuningParams.Wn_altitude  =  math.sqrt(controlGains.ki_altitude * kDC * linearizedModel.Va_trim)
    tuningParams.Zeta_altitude  = (controlGains.kp_altitude * kDC * linearizedModel.Va_trim)/(2* tuningParams.Wn_altitude)


    tuningParams.Wn_SpeedfromThrottle  = math.sqrt(controlGains.ki_SpeedfromThrottle *linearizedModel.a_V2)
    tuningParams.Zeta_SpeedfromThrottle  = (controlGains.kp_SpeedfromThrottle * linearizedModel.a_V2 + linearizedModel.a_V1)/(2* tuningParams.Wn_SpeedfromThrottle)

    """
    controlGains.kp_SpeedfromElevator = (linearizedModel.a_V1 - 2*tuningParameters.Zeta_SpeedfromElevator * tuningParameters.Wn_SpeedfromElevator)/(kDC*VPC.g0)
    controlGains.ki_SpeedfromElevator = -(tuningParameters.Wn_SpeedfromElevator ** 2)/(kDC * VPC.g0)
    """
    tuningParams.Wn_SpeedfromElevator  = math.sqrt(-controlGains.ki_SpeedfromElevator * kDC * VPC.g0)
    tuningParams.Zeta_SpeedfromElevator  = -(controlGains.kp_SpeedfromElevator * kDC*VPC.g0 - linearizedModel.a_V1)/(2*tuningParams.Wn_SpeedfromElevator)
    
    
    return tuningParams


