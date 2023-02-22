import math
import sys
import ece163.Containers.Inputs as Inputs
import ece163.Containers.Controls as Controls
import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Modeling.VehicleAerodynamicsModel as VehicleAerodynamicsModule

class PDControl():

    def __init__(self, kp = 0.0, kd = 0.0, trim = 0.0, lowLimit = 0.0, highLimit = 0.0):
        self.kp = kp
        self.kd = kd
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        return

    def setPDGains(self, kp = 0.0, kd= 0.0, trim= 0.0, lowLimit = 0.0, highLimit = 0.0):
        self.kp = kp
        self.kd = kd
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        return

    def Update(self, command = 0.0, current=0.0, derivative=0.0):
        u = 0.0

        return u


class PIControl():

    def __init__(self):

        return

    def setPIGains(self):

        return

    def Update(self):

        return

    def resetIntegrator(self):

        return

class PIDControl():

    def __init__(self):

        return

    def setPIDGains(self):

        return

    def Update(self):

        return

    def resetIntegrator(self):

        return

class VehicleClosedLoopControl():

    def __init__(self):

        return
    
    def reset(self):

        return
    
    def getControlGains(self):

        return

    def setControlGains(self):

        return

    def getVehicleState(self):

        return

    def setVehicleState(self):

        return 

    def getTrimInputs(self):

        return

    def setTrimInputs(self):

        return 
    
    def getVehicleAerodynamicsModel(self):

        return
    
    def getVehicleControlSurfaces(self):

        return 
    
    def UpdateControlCommands(self):

        return

    def Update(self):

        return