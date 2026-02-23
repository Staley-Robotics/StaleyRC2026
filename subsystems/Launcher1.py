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

class Launcher(Subsystem):
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
        self.desired_speed: rotations_per_second = 0

    def periodic(self) -> None:
        # Logging: Write Current Measured Subsystem State
        FalconLogger.logInput("/Intake/Inputs/launchMotor/velocity", self.motor.get_velocity().value)

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled():
            self.stop()
        else:
            self.run()
        
        # Logging: Write Post Operation Information
        FalconLogger.logOutput("/Intake/Outputs/Setpoint", self.getDesiredSpeed())

    def run(self) -> None:
        # control velocity
        self.motor.set_control(VelocityVoltage, self.desired_speed)

    def stop(self) -> None:
        pass

    def setDesiredSpeed(self, speed:rotations_per_second) -> None:
        self.desired_speed = speed

    def getDesiredSpeed(self) -> rotations_per_second:
        return self.desired_speed
    
    def isAtSpeed(self) -> bool:
        return self.motor.get_closed_loop_error().value