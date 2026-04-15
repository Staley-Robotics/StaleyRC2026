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

import RobotWideConstants

from util.FalconLogger import FalconLogger



class Flywheel(Subsystem):

    def __init__(self, motorID, Constants, isDisabled:typing.Callable[[], bool]) -> None:
        ### Motor Setup
        ## Launch Motor
        self.motor = TalonFX(motorID, "rio")

        self.Constants = Constants
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
        ).with_closed_loop_ramps(
            ClosedLoopRampsConfigs()
                .with_voltage_closed_loop_ramp_period(0.25)
        ).with_current_limits(
            CurrentLimitsConfigs()
            .with_stator_current_limit(self.Constants.statorLimit)
            .with_stator_current_limit_enable(self.Constants.statorLimitEnable)
            .with_supply_current_limit(self.Constants.supplyLimit)
            .with_supply_current_limit_enable(self.Constants.supplyLimitEnable)
            .with_supply_current_lower_limit(self.Constants.supplyLowerLimit)
            .with_supply_current_lower_time(self.Constants.supplyLowerTime)
        ) # hitting limit caused continual crash without clear error?
        self.motor.configurator.apply(motor_config)

        ### Functionality Setup
        self.velocity_req = VelocityVoltage(0.0)

        self.disabled = isDisabled
