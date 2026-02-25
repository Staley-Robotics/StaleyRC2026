from enum import Enum

from commands2 import Subsystem
from wpilib import RobotState
from ntcore.util import ntproperty

from wpimath.units import *
from phoenix6.units import *

from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration, MotorOutputConfigs, Slot0Configs
from phoenix6.signals import InvertedValue, NeutralModeValue
from phoenix6.controls import VoltageOut

from util.FalconLogger import FalconLogger

class Agitator(Subsystem):
    def __init__(self, motorID:int) -> None:
        ### Motor Setup
        ## Launch Motor
        self.motor = TalonFX(motorID, "rio")

        # Config
        motor_config = TalonFXConfiguration()
        motor_config = motor_config.with_motor_output(
            MotorOutputConfigs()
            .with_neutral_mode(NeutralModeValue.COAST)
            .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
        )
        self.motor.configurator.apply(motor_config)

        ### Functionality Setup
        self.motor_volt_req = VoltageOut(0.0)

        # Logging
        FalconLogger.addLoggedObject("Agitator/Inputs/motor", self.motor)

    def periodic(self) -> None:
        # Logging: Write Current Measured Subsystem State

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled():
            self.stop()
        else:
            self.run()
        
        # Logging: Write Post Operation Information
        FalconLogger.logOutput("/Agitator/Outputs/Setpoint", self.getSetSpeed())

    def run(self) -> None:
        # control velocity
        self.motor.set_control(self.motor_volt_req)

    def stop(self) -> None:
        self.motor_volt_req.output = 0.0

    def setSpeed(self, speed:percent) -> None:
        self.motor_volt_req.output = speed * 12

    def getSetSpeed(self) -> percent:
        return self.motor_volt_req.output / 12