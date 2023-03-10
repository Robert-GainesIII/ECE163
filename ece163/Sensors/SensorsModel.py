import math
import random
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Utilities import MatrixMath
from ..Containers import Sensors
from ..Constants import VehiclePhysicalConstants as VPC
from ..Constants import VehicleSensorConstants as VSC
from ..Modeling import VehicleAerodynamicsModel


class GaussMarkov():

    def __init__(self, dT = VPC.dT, tau =1000000, eta=0.0):
        self.dT = dT
        self.tau = tau
        self.eta = eta
        self.v = 0

    def reset(self):
        self.v = 0

    def update(self, vnoise = None):
        newV = 0
        if(vnoise == None):
            vnoise = random.gauss(0, 1)
        
        newV = ((math.exp(-self.dT/self.tau))*self.v)+vnoise

        self.v = newV
        return self.v


class GuassMarkovXYZ():

    def __init__(self, dT = VPC.dT, tauX =1000000, etaX=0.0, tauY =None, etaY=None, tauZ=None, etaZ=None):
        
        self.dT = dT
        self.vX = 0
        self.vY = 0
        self.vZ = 0

        if(tauZ == None and tauY == None):
            #use first set of values for both
            self.tauX = tauX
            self.etaX = etaX
            self.tauY = tauX
            self.etaY = etaX
            self.tauZ = tauX
            self.etaZ = etaX
        elif (tauZ == None and tauY!= None):
            self.tauX = tauX
            self.etaX = etaX
            self.tauY = tauY
            self.etaY = etaY
            self.tauZ = tauY
            self.etaZ = etaY
        else:
            self.tauX = tauX
            self.etaX = etaX
            self.tauY = tauY
            self.etaY = etaY
            self.tauZ = tauZ
            self.etaZ = etaZ

    def reset(self):
        self.vX = 0
        self.vY = 0
        self.vZ = 0

    def update(self, vXnoise = None, vYnoise = None, vZnoise = None):
        new_Vx = 0
        new_Vy = 0
        new_Vz = 0
        if(vXnoise == None):
            vXnoise= random.gauss(0, 1)
        new_Vx = ((math.exp(-self.dT/self.tau))*self.vX)+vXnoise
        self.vX = new_Vx

        if(vYnoise == None):
            vYnoise= random.gauss(0, 1)
        new_Vy = ((math.exp(-self.dT/self.tau))*self.vY)+vYnoise
        self.vY = new_Vy

        if(vZnoise == None):
            vZnoise= random.gauss(0, 1)
        new_Vz = ((math.exp(-self.dT/self.tau))*self.vZ)+vZnoise
        self.vZ = new_Vz



        return new_Vx, new_Vy, new_Vz


class SensorsModel():

    def __init__(self):
        pass

    def initializeBiases(self):
        pass

    def initializeSigmas(self):
        pass

    def updateGPSTrue(self):
        pass

    def updateAccelsTrue(self):
        pass

    def updateMagsTrue(self):
        pass

    def updateGyrosTrue(self):
        pass

    def updatePressureSensorsTrue(self):
        pass

    def updateSensorsTrue(self):
        pass

    def updateSensorsNoisy(self):
        pass

    def update(self):
        pass

    def getSensorsTrue(self):
        pass

    def getSensorsNoisy(self):
        pass

    def reset(self):
        pass