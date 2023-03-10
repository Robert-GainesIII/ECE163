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


class GaussMarkovXYZ():

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
        new_Vx = ((math.exp(-self.dT/self.tauX))*self.vX)+vXnoise
        self.vX = new_Vx

        if(vYnoise == None):
            vYnoise= random.gauss(0, 1)
        new_Vy = ((math.exp(-self.dT/self.tauY))*self.vY)+vYnoise
        self.vY = new_Vy

        if(vZnoise == None):
            vZnoise= random.gauss(0, 1)
        new_Vz = ((math.exp(-self.dT/self.tauZ))*self.vZ)+vZnoise
        self.vZ = new_Vz



        return new_Vx, new_Vy, new_Vz


class SensorsModel():

    def __init__(self,aeroModel=VehicleAerodynamicsModel.VehicleAerodynamicsModel(),taugyro=400.0,etagyro=0.0012740903539558606, tauGPS = 1100.0, etaGPSHorizontal=0.21, etaGPSVertical=0.4, gpsUpdateHz=1.0):
        self.aeroModel = aeroModel
        self.dynamicModel = aeroModel.getVehicleDynamicsModel()
        self.dT = self.dynamicModel.dT
        self.sensorsTrue = Sensors.vehicleSensors()
        self.sensorsNoisy = Sensors.vehicleSensors()
        self.sensorBiases = self.initializeBiases()
        self.sensorSigmas = self.initializeSigmas()
        self.GPS = GaussMarkovXYZ(1/gpsUpdateHz, tauGPS, etaGPSHorizontal, tauGPS, etaGPSHorizontal, tauGPS, etaGPSVertical)
        self.Gyro = GaussMarkovXYZ(dT=self.dT, tauX=taugyro, etaX=etagyro)

    def initializeBiases(self, gyroBias = 0.08726646259971647, accelBias=0.9810000000000001, magBias=500.0, baroBias = 100.0, pitotBias = 20.0):
        sensorBiases = Sensors.vehicleSensors()
        sensorBiases.gyro_x = gyroBias * random.uniform(-1, 1)
        sensorBiases.gyro_y = gyroBias * random.uniform(-1, 1)
        sensorBiases.gyro_z = gyroBias * random.uniform(-1, 1)
        sensorBiases.accel_x = accelBias * random.uniform(-1, 1)
        sensorBiases.accel_y = accelBias * random.uniform(-1, 1)
        sensorBiases.accel_z = accelBias * random.uniform(-1, 1)
        sensorBiases.baro = baroBias * random.uniform(-1,1)
        sensorBiases.pitot = pitotBias * random.uniform(-1,1)
        sensorBiases.mag_x = magBias * random.uniform(-1, 1)
        sensorBiases.mag_y = magBias * random.uniform(-1, 1)
        sensorBiases.mag_z = magBias * random.uniform(-1, 1)


    def initializeSigmas(self, gyroSigma=0.002617993877991494, accelSigma=0.24525000000000002, magSigma=25.0, baroSigma=10.0, pitotSigma=2.0, gpsSigmaHorizontal=0.4, gpsSigmaVertical=0.7,gpsSigmaSOG=0.05, gpsSigmaCOG=0.002):
        sensorSigmas = Sensors.vehicleSensors()
        sensorSigmas.gyro_x = gyroSigma
        sensorSigmas.gyro_y = gyroSigma
        sensorSigmas.gyro_z = gyroSigma
  
        sensorSigmas.accel_x = accelSigma
        sensorSigmas.accel_y = accelSigma
        sensorSigmas.accel_z = accelSigma
        sensorSigmas.baro = baroSigma
        sensorSigmas.pitot = pitotSigma
        sensorSigmas.mag_x = magSigma
        sensorSigmas.mag_y = magSigma
        sensorSigmas.mag_z = magSigma
        
    


    def updateGPSTrue(self, state, dot):
        GPS = [state.pn,state.pe,-state.pd,math.hypot(dot.pn, dot.pe),math.atan2(dot.pe, dot.pn)]
        
        return GPS

    def updateAccelsTrue(self, state, dot):
        forces = [0,0,0]
        velocity_derivatives = [[dot.u],[dot.v],[dot.w]]
        skew = [[0.0,-state.r, state.q],[state.q, 0.0, -state.p],[-state.r, state.p, 0.0]]
        v = [[state.u],[state.v],[state.w]]
        gravityBVec = MatrixMath.multiply(state.R, [[0],[0],[VPC.g0]])

        skewTimesV = MatrixMath.multiply(skew, v)
        aMeas = MatrixMath.subtract(MatrixMath.add(velocity_derivatives, skewTimesV), gravityBVec)
        forces[0] = aMeas[0][0]
        forces[1] = aMeas[1][0]
        forces[2] = aMeas[2][0]
        return forces

    def updateMagsTrue(self,state):
        forces = MatrixMath.multiply(state.R, VSC.magfield)

        return [forces[0][0], forces[1][0], forces[2][0]]

    def updateGyrosTrue(self,state):
        forces = [state.p, state.q, state.r]

        return forces

    def updatePressureSensorsTrue(self, state):
        hbaro = VPC.rho * VPC.g0  * state.pn
        pitot = VPC.rho * (state.Va**2/2)

        return [hbaro]

    def updateSensorsTrue(self,prevTrueSensors, state, dot):
        #no noise or biases
        trueOutputs = Sensors.vehicleSensors()

        gyroT = self.updateGyrosTrue(state)
        trueOutputs.gyro_x = gyroT[0]
        trueOutputs.gyro_y = gyroT[1]
        trueOutputs.gyro_z = gyroT[2]

        accelT = self.updateAccelsTrue(state, dot)
        trueOutputs.accel_x = accelT [0]
        trueOutputs.accel_y = accelT[1]
        trueOutputs.accel_z = accelT[2]

        baroT = self.updatePressureSensorsTrue(state)
        trueOutputs.baro = baroT[0]
        trueOutputs.gps_alt = baroT[0]

        gpsT = self.updateGPSTrue(state, dot)
        
        trueOutputs.gps_n = gpsT[0]
        trueOutputs.gps_e = gpsT[1]
        trueOutputs.gps_alt= gpsT[2]
        trueOutputs.gps_sog= gpsT[3]
        trueOutputs.gps_cog= gpsT[4]

        magT = self.updateMagsTrue(state)
        trueOutputs.mag_x = magT[0]
        trueOutputs.mag_y = magT[1]
        trueOutputs.mag_z = magT[2]

        trueOutputs.pitot = 5.706900

        
        return trueOutputs

    def updateSensorsNoisy(self,trueSensors=Sensors.vehicleSensors(gyro_x=0.0, gyro_y=0.0, gyro_z=0.0, accel_x=0.0, accel_y=0.0, accel_z=0.0, mag_x=0.0, mag_y=0.0, mag_z=0.0, baro=0.0, pitot=0.0, gps_n=0.0, gps_e=0.0, gps_alt=0.0, gps_sog=0.0, gps_cog=0.0),
                                noisySensors=Sensors.vehicleSensors(gyro_x=0.0, gyro_y=0.0, gyro_z=0.0, accel_x=0.0,accel_y=0.0, accel_z=0.0, mag_x=0.0, mag_y=0.0, mag_z=0.0, baro=0.0, pitot=0.0,gps_n=0.0, gps_e=0.0, gps_alt=0.0, gps_sog=0.0, gps_cog=0.0),
                                sensorBiases=Sensors.vehicleSensors(gyro_x=0.0, gyro_y=0.0, gyro_z=0.0, accel_x=0.0,accel_y=0.0, accel_z=0.0, mag_x=0.0, mag_y=0.0, mag_z=0.0, baro=0.0, pitot=0.0,gps_n=0.0, gps_e=0.0, gps_alt=0.0, gps_sog=0.0, gps_cog=0.0),
                                sensorSigmas=Sensors.vehicleSensors(gyro_x=0.0, gyro_y=0.0, gyro_z=0.0, accel_x=0.0,accel_y=0.0, accel_z=0.0, mag_x=0.0, mag_y=0.0, mag_z=0.0, baro=0.0, pitot=0.0,gps_n=0.0, gps_e=0.0, gps_alt=0.0, gps_sog=0.0, gps_cog=0.0)):
        noisyData = Sensors.vehicleSensors()

        return noisyData

    def update(self):
        
        return 

    def getSensorsTrue(self):
        
        return self.sensorsTrue

    def getSensorsNoisy(self):
        
        return self.sensorsNoisy

    def reset(self):
        

        return