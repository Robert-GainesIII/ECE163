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
    linearizedModel.kp_roll = tuningParameters.Wn_roll ** 2 / linearizedModel.a_phi2
    linearizedModel.kd_roll = ((2* tuningParameters.Zeta_roll * tuningParameters.Wn_roll) - linearizedModel.a_phi1)/linearizedModel.a_phi2
    linearizedModel.ki_roll = 0
    linearizedModel.kp_sideslip = 0
    linearizedModel.ki_sideslip = 0
    linearizedModel.kp_course = 0
    linearizedModel.ki_course = 0
    # Longitudinal Gains
    linearizedModel.kp_pitch = 0
    linearizedModel.kd_pitch = 0
    linearizedModel.kp_altitude = 0
    linearizedModel.ki_altitude = 0
    linearizedModel.kp_SpeedfromThrottle = 0
    linearizedModel.ki_SpeedfromThrottle = 0
    linearizedModel.kp_SpeedfromElevator = 0
    linearizedModel.ki_SpeedfromElevator = 0
    return controlGains

def computeTuningParameters(controlGains, linearizedModel):

    tuningParams = Controls.controlTuning()
    """
    
    tuningParams.Wn_roll  = 20
    tuningParams.Zeta_roll  = math.sqrt(2)
    tuningParams.Wn_course  = 1	# Wn_roll should be 5-10x larger
    tuningParams.Zeta_course  = 1
    tuningParams.Wn_sideslip  = 1
    tuningParams.Zeta_sideslip  = 1
    #tuning knobs for longitudinal control
    tuningParams.Wn_pitch  = 0
    tuningParams.Zeta_pitch  = 0
    tuningParams.Wn_altitude  = 0	# Wn_pitch should be 5-10x larger
    tuningParams.Zeta_altitude  = 0 
    tuningParams.Wn_SpeedfromThrottle  = 0
    tuningParams.Zeta_SpeedfromThrottle  = 0
    tuningParams.Wn_SpeedfromElevator  = 0
    tuningParams.Zeta_SpeedfromElevator  = 0
    
    """
    return tuningParams


