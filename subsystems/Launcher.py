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

class Launcher(Subsystem):
    '''This is functionally quite similar (if not the same) as Agitator, but for now they are kept seperate for a variety of annoyances' sake'''
    class LauncherSpeeds:
        WAIT:rotations_per_second = 10 # default speed for lower power consumption but faster acceleration when needed
        SPEED_AT_ZERO_DIST:rotations_per_second = 20 # total guess, speed at minimum distance TODO: measure
        SPEED_AT_MAX_DIST:rotations_per_second = 70 # total guess, speed at some arbitrary larger distance TODO: measure
        SPEED_LOW:rotations_per_second = 20
        SPEED_MED:rotations_per_second = 40
        SPEED_HIGH:rotations_per_second = 70

        EJECT:rotations_per_second = 13 # just enough to overshoot the intake

        STOP:rotations_per_second = 0 # at 5 to keep moving and reduce acceleration later

        '''
        kraken free speed max: 6000 rpm = 100 rps
        measured mechanism speed at 12 volts = ~83
        cut to 75 for safety (and because free speed is gonna be higher than max in our mechanism)

        NOTE: actual flywheel speed will be double the motor speed because of gearing
        '''
        kMaxAllowedSpeed:rotations_per_second = 75.0
        kSpeedAt12Volts:rotations_per_second = 83.0
    
    class LauncherDistances:
        MIN:meters=1
        MAX:meters=10

    class Constants:
        k_P:float=0.7
        k_I:float=0.0
        k_D:float=0.03
        k_S:float=0.22
        k_V:float=0.11

    kAtSpeedTolerance:rotations_per_second = 6.0 #ntproperty("/Settings/Launcher/atSpeed tolerance", 2.0, persistent=True)

    # disabled = ntproperty("/Disabling/Launcher", False, persistent=False)

    def __init__(self, motorID:int, isDisabled:typing.Callable[[], bool]) -> None:
        ### Motor Setup
        ## Launch Motor
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
        ).with_closed_loop_ramps(
            ClosedLoopRampsConfigs()
                .with_voltage_closed_loop_ramp_period(0.25)
        ).with_current_limits(
            CurrentLimitsConfigs()
            .with_stator_current_limit(100.0)
            .with_stator_current_limit_enable(True)
            .with_supply_current_limit(30)
            .with_supply_current_limit_enable(True)
            .with_supply_current_lower_limit(30)
            .with_supply_current_lower_time(1.0)
        ) # hitting limit caused continual crash without clear error?
        self.motor.configurator.apply(motor_config)

        ### Functionality Setup
        self.velocity_req = VelocityVoltage(0.0)

        self.disabled = isDisabled

        # Logging
        FalconLogger.addLoggedObject("Launcher/Inputs/motor", self.motor)

    def periodic(self) -> None:
        # Logging: Write Current Measured Subsystem State

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled() or self.disabled():
            self.stop()
        else:
            self.run()
        
        # Logging: Write Post Operation Information
        FalconLogger.logOutput("/Launcher/Outputs/Setpoint", self.getDesiredSpeed())
        FalconLogger.logOutput("/Launcher/Outputs/isAtSpeed", self.isAtSpeed())

        FalconLogger.logOutput("systemStates/Launcher running", self.isAtSpeed())
        FalconLogger.logOutput("/Disabling/Launcher", self.disabled())

    def run(self) -> None:
        # control velocity
        if self.getDesiredSpeed() == 0 and not self.disabled():
            self.motor.set(0)
        else:
            self.motor.set_control(self.velocity_req)

    def stop(self) -> None:
        self.velocity_req.velocity = 0.0
        self.motor.set(0)

    # def toggleDisabled(self) -> None:
    #     self.disabled = not self.disabled

    def setDesiredSpeed(self, speed:rotations_per_second) -> None:
        self.velocity_req.velocity = min(speed, self.LauncherSpeeds.kMaxAllowedSpeed)
    
    def getCurrentSpeed(self) -> rotations_per_second:
        return self.motor.get_velocity().value
    def getDesiredSpeed(self) -> rotations_per_second:
        return self.velocity_req.velocity
    
    def isAtSpeed(self) -> bool:
        return abs(self.getDesiredSpeed() - self.getCurrentSpeed()) < self.kAtSpeedTolerance