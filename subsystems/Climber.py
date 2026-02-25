from enum import Enum

from commands2 import Subsystem
from wpilib import RobotState
from ntcore.util import ntproperty

from wpimath.units import *
from phoenix6.units import *

from rev import SparkMax, SparkBase, SparkMaxConfig, ClosedLoopConfig, ClosedLoopSlot, SparkMaxSim, LimitSwitchConfig, EncoderConfig, ResetMode, PersistMode

from util.FalconLogger import FalconLogger

class ClimberConstants:
    _kP = 0.0
    _kI = 0.0
    _kD = 0.0
    _kG = 0.0   # force to overcome gravity
    _kS = 0.0   # force to overcome friction
    _kV = 0.0   # Apply __ voltage for target velocity
    _kFF = 0.0  # Feed Forward

    _kOffset = 0.0
    _kAtSetpointTolerance:inches = 1.0

    _pulleyDiameter:inches = 0.880
    _pulleyRadius:inches = 0.880 / 2

    _gearRatio = 100.0
    _carriageMass:kilograms = 1.0 # Very rough estimate

    _motorRotsPerHeightInches =  1 / _gearRatio * (_pulleyDiameter * math.pi) # rotations / height


class Climber(Subsystem):

    class ClimberPositions:
        MAX:inches = 8.0
        MIN:inches = 0.0

    def __init__(self, motorID:int) -> None:
        ### Motor Setup
        ## Launch Motor
        self.motor = SparkMax(motorID, SparkMax.MotorType.kBrushless)

        # retrieve motor objs
        self.motor_encoder = self.motor.getAbsoluteEncoder()
        self.pid_controller = self.motor.getClosedLoopController()

        # Config
        motorCfg = SparkMaxConfig()\
            .setIdleMode( SparkMaxConfig.IdleMode.kBrake )\
            .inverted( True )

        clCfg = ClosedLoopConfig()\
            .pidf(
                ClimberConstants._kP,
                ClimberConstants._kI,
                ClimberConstants._kD,
                ClimberConstants._kFF,
                ClosedLoopSlot.kSlot0
            )\
            .positionWrappingEnabled( False )

        convFactor = ClimberConstants._motorRotsPerHeightInches
        encConfig = EncoderConfig()\
            .positionConversionFactor( convFactor ).velocityConversionFactor( convFactor / 60)

        #NOTE: motor forwards = claw down
        lsConfig = LimitSwitchConfig()\
            .reverseLimitSwitchEnabled(True)\
            .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)\
            .forwardLimitSwitchEnabled(True)\
            .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)\
            
            # .reverseLimitSwitchPosition()\
            # .forwardLimitSwitchPosition()

        # Apply Configs
        motorCfg.apply(clCfg)
        motorCfg.apply(encConfig)
        motorCfg.apply(lsConfig)

        self.motor.configure(motorCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

        ### Functionality Setup
        self.desired_position: inches = self.ClimberPositions.MIN

        ### Logging Setup
        FalconLogger.addLoggedObject("/Climber/motor", self.motor)

    def periodic(self) -> None:
        # Logging: Write Current Measured Subsystem State
        

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled():
            self.stop()
        else:
            self.run()
        
        # Logging: Write Post Operation Information
        FalconLogger.logOutput("/Climber/Outputs/desired position", self.desired_position)

    def run(self) -> None:
        # update desired position
        self.pid_controller.setReference(
            self.desired_position,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0
        )

    def stop(self) -> None:
        self.setDesiredPosition(self.getHeight())

    ## External Funcs
    def setDesiredPosition(self, pos:inches) -> None:
        '''
        Sets the setpoint sent to the pid controll to the `pos` param
        restricts inputs to min/max of the climber mechanism
        '''
        self.desired_position = (max(min(pos, self.ClimberPositions.MAX), self.ClimberPositions.MIN))

    def getDesiredPosition(self) -> inches:
        return self.desired_position
    
    def getHeight(self) -> inches:
        return self.motor_encoder.getPosition()
    
    def isAtDesiredPosition(self) -> bool:
        return abs(self.getHeight() - self.getDesiredPosition()) < ClimberConstants._kAtSetpointTolerance