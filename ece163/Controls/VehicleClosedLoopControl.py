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
        self.differentiator = 0.0
        self.err = 0.0
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
        error = command - current
        #if(self.accumulator < self.highLimit and self.accumulator > self.lowLimit):
        #    self.accumulator += 0.5 * self.dT * (command-current + self.err)
        
        #derivative = ((2* math.tau - VPC.dT) / (2 * math.tau + VPC.dT)) * derivative + (2 / (2 *math.tau + VPC.dT))
        u = (self.kp * error) - (self.kd * derivative ) + self.trim

        if u < self.lowLimit:
            u = self.lowLimit
        if u > self.highLimit:
            u = self.highLimit 
        
        self.err = error

        return u


class PIControl():

    def __init__(self, dT =VPC.dT,  kp = 0.0, ki = 0.0, trim = 0.0, lowLimit = 0.0, highLimit = 0.0):
        self.dT = dT
        self.kp = kp
        self.ki = ki
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        self.accumulator = 0.0
        self.err = 0.0
        return

    def setPIGains(self, dT =VPC.dT , kp = 0.0, ki = 0.0, trim = 0.0, lowLimit = 0.0, highLimit = 0.0):
        self.dT = dT
        self.kp = kp
        self.ki = ki
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        return

    def Update(self, command = 0.0, current=0.0):
        u = 0.0
        error = command - current



        self.accumulator += (0.5 * self.dT) * (error + self.err)

        #derivative = (2*math.tau -self.dT)/(2*math.tau + self.dT) * derivative + (2/(2*math.tau + self.dT)) *  (error - self.err)
        
        u_u= (self.kp * error) + (self.ki * self.accumulator)  + self.trim
        u = u_u
        if u < self.lowLimit:
            u = self.lowLimit
            self.accumulator -= 0.5 * self.dT * (error + self.err)
        if u > self.highLimit:
            u = self.highLimit
            self.accumulator -= 0.5 * self.dT * (error + self.err)


        self.err = error
        
        return u

    def resetIntegrator(self):
        self.accumulator = 0.0
        self.err = 0.0
        return

class PIDControl():

    def __init__(self, dT =VPC.dT,  kp = 0.0, kd=0.0, ki = 0.0, trim = 0.0, lowLimit = 0.0, highLimit = 0.0):
        self.dT = dT
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        self.accumulator = 0.0
        self.differentiator = 0.0
        self.err = 0.0
        self.flag = 0
        return

    def setPIDGains(self, dT =VPC.dT,  kp = 0.0, kd=0.0, ki = 0.0, trim = 0.0, lowLimit = 0.0, highLimit = 0.0):
        self.dT = dT
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        return

    def Update(self, command=0.0, current=0.0, derivative=0.0):
        u = 0.0
        error = command - current



        self.accumulator += (0.5 * self.dT) * (error + self.err)

        #derivative = (2*math.tau -self.dT)/(2*math.tau + self.dT) * derivative + (2/(2*math.tau + self.dT)) *  (error - self.err)
        
        u_u= (self.kp * error) + (self.ki * self.accumulator) - (self.kd * derivative) + self.trim
        u = u_u
        if u < self.lowLimit:
            u = self.lowLimit
            self.accumulator -= 0.5 * self.dT * (error + self.err)
        if u > self.highLimit:
            u = self.highLimit
            self.accumulator -= 0.5 * self.dT * (error + self.err)

        self.err = error
        
        return u

    def resetIntegrator(self):
        self.accumulator = 0.0
        self.err = 0.0
        return

class VehicleClosedLoopControl():

    def __init__(self, dT = VPC.dT, rudderControlSource='SLIDESLIP'):
        self.dT = dT
        self.rudderControlSource = rudderControlSource #EITHER SLIDESLIP OR YAW
        self.VAM = VehicleAerodynamicsModule.VehicleAerodynamicsModel()
        self.GAINS = Controls.controlGains()
        self.TRIM_CONTROL_INPUT_CONTAINER = Inputs.controlInputs()
        self.VAM_INPUT_FROM_OUTPUT_CONTAINER= Inputs.controlInputs()
        self.altitudeState = Controls.AltitudeStates(Controls.AltitudeStates.HOLDING)
        self.rollFromCourse = PIControl()
        self.rudderFromSideslip = PIControl()
        self.throttleFromAirspeed = PIControl()
        self.pitchFromAltitude = PIControl()
        self.pitchFromAirspeed = PIControl()
        self.elevatorFromPitch = PDControl()
        self.aileronFromRoll = PIDControl()
        return
    
    def reset(self):
        
        self.rollFromCourse.resetIntegrator()
        self.rudderFromSideslip.resetIntegrator()
        self.throttleFromAirspeed.resetIntegrator()
        self.pitchFromAltitude.resetIntegrator()
        self.pitchFromAirspeed.resetIntegrator()
        self.aileronFromRoll.resetIntegrator()
        self.VAM.reset()
        return
    
    def getControlGains(self):
     
        return self.GAINS

    def setControlGains(self, controlGains = Controls.controlGains()):
        self.GAINS = controlGains
        self.rollFromCourse.setPIGains(self.dT, self.GAINS.kp_course, self.GAINS.ki_course, 0.0, -math.radians(VPC.bankAngleLimit), math.radians(VPC.bankAngleLimit))
        self.rudderFromSideslip.setPIGains(self.dT, self.GAINS.kp_sideslip, self.GAINS.ki_sideslip, self.trimInputs.Rudder, VPC.minControls.Rudder, VPC.maxControls.Rudder)
        self.throttleFromAirspeed.setPIGains(self.dT, self.GAINS.kp_SpeedfromThrottle, self.GAINS.ki_SpeedfromThrottle, self.trimInputs.Throttle, VPC.minControls.Throttle, VPC.maxControls.Throttle)
        self.pitchFromAltitude.setPIGains(self.dT, self.GAINS.kp_altitude, self.GAINS.ki_altitude, 0.0, -math.radians(VPC.pitchAngleLimit), math.radians(VPC.pitchAngleLimit))
        self.pitchFromAirspeed.setPIGains(self.dT, self.GAINS.kp_SpeedfromElevator, self.GAINS.ki_SpeedfromElevator, 0.0, -math.radians(VPC.pitchAngleLimit),math.radians(VPC.pitchAngleLimit))
        self.elevatorFromPitch.setPDGains(self.GAINS.kp_pitch, self.GAINS.kd_pitch, self.trimInputs.Elevator, VPC.minControls.Elevator, VPC.maxControls.Elevator)
        self.aileronFromRoll.setPIDGains(self.dT, self.GAINS.kp_roll, self.GAINS.kd_roll, self.GAINS.ki_roll, self.trimInputs.Aileron, VPC.minControls.Aileron, VPC.maxControls.Aileron)
        return

    def getVehicleState(self):

        return self.VAM.getVehicleState()

    def setVehicleState(self, state):
        self.state = state
        return 

    def getTrimInputs(self):

        return self.trimInputs

    def setTrimInputs(self, trimInputs=Inputs.controlInputs(Throttle=0.5, Aileron=0.0, Elevator=0.0, Rudder=0.0)):

        self.trimInputs = trimInputs
        return 
    
    def getVehicleAerodynamicsModel(self):
        
        return self.VAM
    
    def getVehicleControlSurfaces(self):
        
        return Inputs.controlInputs(Throttle=self.throttleFromAirspeed , Aileron=self.aileronFromRoll, Elevator=self.elevatorFromPitch, Rudder=self.rudderFromSideslip )
    
    def UpdateControlCommands(self, referenceCommands, state):
        #state = self.getVehicleState()
        inputs = Inputs.controlInputs()
        if state.chi > math.pi:
            state.chi += math.pi*2.0
        elif state.chi < -math.pi:
            state.chi -= math.pi*2.0

        lower_thresh = referenceCommands.commandedAltitude - VPC.altitudeHoldZone
        upper_thresh = referenceCommands.commandedAltitude + VPC.altitudeHoldZone
        alt = -state.pd 

        airThrottle = self.throttleFromAirspeed.Update(referenceCommands.commandedAirspeed, state.Va)
        airPitch = self.pitchFromAirspeed.Update(referenceCommands.commandedAirspeed, state.Va)
        altPitch = self.pitchFromAltitude.Update(referenceCommands.commandedAltitude, alt)
        
        x = referenceCommands.commandedPitch
        #BEGINNING OF STATE MACHINE
        #STATE = HOLDING
        if self.altitudeState == Controls.AltitudeStates.HOLDING:
            inputs.Throttle = self.throttleFromAirspeed.Update(referenceCommands, state.Va)
            #TRANSISTION FROM HOLDING TO DESCENDING
            if alt > upper_thresh:
                x = airPitch
                inputs.Throttle = VPC.minControls.Throttle
                self.altitudeState = Controls.AltitudeStates.DESCENDING
                self.pitchFromAirspeed.resetIntegrator()
            
            #TRANSISTION FROM HOLDING TO CLIMBING
            if alt < lower_thresh: 
                x = airPitch
                inputs.Throttle = VPC.maxControls.Throttle
                self.altitudeState = Controls.AltitudeStates.CLIMBING
                self.pitchFromAirspeed.resetIntegrator()

        #STATE = DESCENDING   
        elif self.altitudeState == Controls.AltitudeStates.DESCENDING:
            
            #TRANSISTION FROM DESCENDING TO HOLDING 
            if alt > lower_thresh and alt < upper_thresh:
                x = altPitch
                inputs.Throttle = self.throttleFromAirspeed.Update(referenceCommands.commandedAirspeed,state.Va)
                self.altitudeState = Controls.AltitudeStates.HOLDING
                self.pitchFromAltitude.resetIntegrator()

        #STATE = CLIMBING
        elif self.altitudeState == Controls.AltitudeStates.CLIMBING:
           
            #TRANSISTION FROM CLIMBING TO HOLDING
            if alt > lower_thresh and alt < upper_thresh:
                x = airPitch
                inputs.Throttle = self.throttleFromAirspeed.Update(referenceCommands.commandedAirspeed,state.Va)
                self.altitudeState = Controls.AltitudeStates.HOLDING
                self.pitchFromAltitude.resetIntegrator()

        else:
            print("this print statement can only mean basd things")

        referenceCommands.commandedPitch = x
        referenceCommands.commandedRoll = self.rollFromCourse.Update(referenceCommands.commandedCourse, state.chi)
        inputs.Elevator = self.elevatorFromPitch.Update(referenceCommands.commandedPitch, state.pitch, state.q)
        
        inputs.Aileron = self.aileronFromRoll.Update(referenceCommands.commandedRoll, state.roll, state.p)
        inputs.Rudder = self.rudderFromSideslip.Update(0.0, state.beta)
        #NOW TO UPDATE COMMANDS THAT DONT DEPEND ON THE STATE
        
        return inputs 

    def Update(self, referenceCommands = Controls.referenceCommands):
        
        inputs = self.UpdateControlCommands(referenceCommands,self.getVehicleState())
        self.VAM.Update(inputs)
        return