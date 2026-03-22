from enum import Enum

from commands2 import Subsystem
from wpilib import RobotState
from ntcore.util import ntproperty

from wpimath.units import *
from phoenix6.units import *

from rev import SparkMax, SparkBase, SparkMaxConfig, ClosedLoopConfig, ClosedLoopSlot, SparkMaxSim, LimitSwitchConfig, EncoderConfig, ResetMode, PersistMode

from util.FalconLogger import FalconLogger

class ClimberConstants:
    _kAtSetpointTolerance:inches = 1.0

    _pulleyDiameter:inches = 0.880
    _pulleyRadius:inches = 0.880 / 2

    _gearRatio = 100.0
    _carriageMass:kilograms = 1.0 # Very rough estimate

    _motorRotsPerHeightInches =  1 / _gearRatio * (_pulleyDiameter * math.pi) # rotations / height


class ClimberOpenLoop(Subsystem):

    class ClimberPositions:
        '''only estimates, atm'''
        MAX:inches = 8.0
        MIN:inches = 0.0
    
    class ClimberSpeeds:
        '''
        NOTE: positive = hook up relative to the robot
        '''
        DEPLOY:percent  = -0.5
        CLIMB:percent   = 0.5
        UNCLIMB:percent = -0.3

    def __init__(self, motorID:int) -> None:
        ### Motor Setup
        ## Launch Motor
        self.motor = SparkMax(motorID, SparkMax.MotorType.kBrushless)

        # retrieve motor objs
        self.motor_encoder = self.motor.getAbsoluteEncoder()

        # Config
        motorCfg = SparkMaxConfig()\
            .setIdleMode( SparkMaxConfig.IdleMode.kBrake )\
            .inverted( True )

        convFactor = ClimberConstants._motorRotsPerHeightInches
        encConfig = EncoderConfig()\
            .positionConversionFactor( convFactor )#.velocityConversionFactor( convFactor / 60) # - velocity deliberately left out

        #NOTE: motor forwards = claw down
        # lsConfig = LimitSwitchConfig()\
        #     .reverseLimitSwitchEnabled(True)\
        #     .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)\
        #     .forwardLimitSwitchEnabled(True)\
        #     .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)\
            
            # .reverseLimitSwitchPosition()\
            # .forwardLimitSwitchPosition()

        # Apply Configs
        motorCfg.apply(encConfig)
        # motorCfg.apply(lsConfig)

        self.motor.configure(motorCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

        ### Functionality Setup
        self.set_speed:percent = 0.0

        ### Logging
        FalconLogger.addLoggedObject("/Climber/motor", self.motor)

    def periodic(self) -> None:
        # Logging: Write Current Measured Subsystem State
        

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled():
            self.stop()
        else:
            self.run()
        
        # Logging: Write Post Operation Information
        FalconLogger.logOutput("/Climber/Outputs/set speed (percent)", self.getSetSpeed())

    def run(self) -> None:
        # apply motor control
        self.motor.set(self.set_speed)

    def stop(self) -> None:
        self.setSpeed(0.0)

    ## External Funcs
    def setSpeed(self, speed:ClimberSpeeds|percent) -> None:
        '''
        Sets the duty cycle (percent) output of the motor
        restricts inputs to [-1,+1]
        '''
        self.set_speed = (max(min(speed, 1), -1))

    def getSetSpeed(self) -> inches:
        return self.set_speed
    
    def getHeight(self) -> inches:
        return self.motor_encoder.getPosition()

    def getVelocity(self) -> rotations_per_second:
        return self.motor_encoder.getVelocity()