from enum import Enum

from commands2 import Subsystem
from wpilib import RobotState
from ntcore.util import ntproperty

from wpimath.units import *
from phoenix6.units import *

from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration, MotorOutputConfigs, Slot0Configs
from phoenix6.signals import InvertedValue, NeutralModeValue
from phoenix6.controls import VelocityVoltage

from util.FalconLogger import FalconLogger

class LauncherConstants:
    kAtSpeedTolerance:rotations_per_second = 2 #total guess

    kMaxExpectedSpeed:rotations_per_second = 50 # 6000rpm is max free speed on specs

class Flywheel(Subsystem):

    class LauncherSpeeds:
        WAIT:rotations_per_second = 5 # default speed for lower power consumption but faster acceleration when needed
        SPEED_AT_ZERO_DIST: 20 # total guess, speed at minimum distance TODO: measure
        SPEED_AT__DIST: 50 # total guess, speed at some arbitrary larger distance TODO: measure

    def __init__(self, launchMotorID:int) -> None:
        ### Motor Setup
        ## Launch Motor
        self.motor = TalonFX(launchMotorID, "rio")

        # Config
        motor_config = TalonFXConfiguration()
        motor_config = motor_config.with_motor_output(
            MotorOutputConfigs()
            .with_neutral_mode(NeutralModeValue.COAST)
            .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
        )
        self.motor.configurator.apply(motor_config)

        ### Functionality Setup
        self.velocity_req = VelocityVoltage(0.0)

        # Logging
        FalconLogger.addLoggedObject("Launcher/Inputs/motor", self.motor)

    def periodic(self) -> None:
        # Logging: Write Current Measured Subsystem State

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled():
            self.stop()
        else:
            self.run()
        
        # Logging: Write Post Operation Information
        FalconLogger.logOutput("/Launcher/Outputs/Setpoint", self.getDesiredSpeed())

    def run(self) -> None:
        # control velocity
        self.motor.set_control(self.velocity_req)

    def stop(self) -> None:
        self.velocity_req.velocity = 0.0

    def setDesiredSpeed(self, speed:rotations_per_second) -> None:
        self.velocity_req.velocity = speed

    def getDesiredSpeed(self) -> rotations_per_second:
        return self.velocity_req.velocity
    
    def isAtSpeed(self) -> bool:
        return abs(self.motor.get_closed_loop_error().value) < LauncherConstants.kAtSpeedTolerance