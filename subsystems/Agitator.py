from enum import Enum

from commands2 import Subsystem
from wpilib import RobotState
from ntcore.util import ntproperty

from wpimath.units import *
from phoenix6.units import *

from phoenix6.hardware import TalonFX
from phoenix6.configs import * #TalonFXConfiguration, MotorOutputConfigs, Slot0Configs, ClosedLoopGeneralConfigs, CurrentLimitsConfigs
from phoenix6.signals import InvertedValue, NeutralModeValue
from phoenix6.controls import VelocityVoltage

from util.FalconLogger import FalconLogger

__all__ = ["Agitator"]

class Agitator(Flywheel):
    class Speeds:
        SPEED_LOW:rotations_per_second = 40
        SPEED_MED:rotations_per_second = 40
        SPEED_HIGH:rotations_per_second = 40
        EJECT:rotations_per_second = -20

    class Constants:
        k_P:float=0.0
        k_I:float=0.0
        k_D:float=0.0
        k_S:float=0.22
        k_V:float=0.112

        kAtSpeedTolerance:rotations_per_second = 3.0

        kMaxAllowedSpeed:rotations_per_second = 60.0
        kSpeedAt12Volts:rotations_per_second = 90.0
class Flywheel(Subsystem):
    '''This is functionally quite similar (if not the same) as Launcher, but for now they are kept seperate for simplicity's sake'''
    
    def __init__(self, motorID:int, isDisabled:typing.Callable[[], bool]) -> None:
        ### Motor Setup
        # Motor object
        self.motor = TalonFX(motorID, "rio")

        # Config
        motor_config = TalonFXConfiguration()
        motor_config = motor_config.with_motor_output(
            MotorOutputConfigs()
            .with_neutral_mode(NeutralModeValue.COAST)
            .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
        ).with_slot0(
            Slot0Configs()\
                .with_k_p(self.Constants.k_P)\
                .with_k_i(self.Constants.k_I)\
                .with_k_d(self.Constants.k_D)\
                .with_k_s(self.Constants.k_S)\
                .with_k_v(self.Constants.k_V)
        ).with_current_limits(
            CurrentLimitsConfigs()
            .with_stator_current_limit(60.0)
            .with_stator_current_limit_enable(True)
            .with_supply_current_limit(20)
            .with_supply_current_limit_enable(True)
            .with_supply_current_lower_limit(20)
            .with_supply_current_lower_time(1.0)
        )

        self.motor.configurator.apply(motor_config)

        ### Functionality Setup
        self.velocity_req = VelocityVoltage(0.0)

        self.disabled = isDisabled

        # Input Logging
        FalconLogger.addLoggedObject("Agitator/Inputs/motor", self.motor)

    def periodic(self) -> None:
        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled() or self.disabled():
            self.stop()
        else:
            self.run()
        
        # Logging: Write Post Operation Information
        FalconLogger.logOutput("/Agitator/Outputs/Setpoint", self.getDesiredSpeed())
        FalconLogger.logOutput("/Launcher/Outputs/isAtSpeed", self.isAtSpeed())

        FalconLogger.logOutput("systemStates/Agitator running", self.getDesiredSpeed() > 5)
        FalconLogger.logOutput("/Disabling/Agitator", self.disabled())

    def run(self) -> None:
        # set velocity control
        self.motor.set_control(self.velocity_req)

    def stop(self) -> None:
        # sets desired speed to 0 for when reenabled
        self.velocity_req.velocity = 0.0
        # sets real motor output to none for slow deceleration
        self.motor.set(0)

    def setDesiredSpeed(self, speed:rotations_per_second) -> None:
        self.velocity_req.velocity = speed

    def getDesiredSpeed(self) -> rotations_per_second:
        return self.velocity_req.velocity
    
    def isAtSpeed(self) -> bool:
        return abs(self.motor.get_closed_loop_error().value) < self.Constants.kAtSpeedTolerance