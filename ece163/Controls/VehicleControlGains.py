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
    
    return controlGains

def computeTuningParameters(controlGains, linearizedModel):

    tuningParams = Controls.controlTuning()
    """
    
    tuningParams.Wn_roll  = 0
    tuningParams.Zeta_roll  = 0
    tuningParams.Wn_course  = 0	# Wn_roll should be 5-10x larger
    tuningParams.Zeta_course  = 0 
    tuningParams.Wn_sideslip  = 0
    tuningParams.Zeta_sideslip  = 0
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


