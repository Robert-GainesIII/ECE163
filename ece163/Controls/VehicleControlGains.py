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

    return tuningParams
    


