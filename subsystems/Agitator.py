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

class Agitator(Subsystem):
    '''This is functionally quite similar (if not the same) as Agitator, but for now they are kept seperate for simplicity's sake'''
    class Speeds:
        WAIT:rotations_per_second = 5 # default speed for lower power consumption but faster acceleration when needed
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

        kAtSpeedTolerance:rotations_per_second = 3.0 # ntproperty("/Agitator/At speed tolerance (rps)", 2.0, persistent=True) #total guess

        '''
        kraken free speed max: 6000 rpm = 100 rps
        cut to 70 for safety (and because free speed is gonna be higher than max in our mechanism)

        NOTE: actual flywheel speed will be double the motor speed because of gearing
        '''
        kMaxAllowedSpeed:rotations_per_second = 60.0
        kSpeedAt12Volts:rotations_per_second = 90.0
    
    disabled = ntproperty("/Disabling/Agitator", False, persistent=False)

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
            .with_supply_current_lower_limit(40)
            .with_supply_current_lower_time(1.0)
        )
        # .with_current_limits(
        #     CurrentLimitsConfigs()
        #     .with_stator_current_limit(80.0)
        #     .with_stator_current_limit_enable(True)
        # ) # hitting limit caused continual crash without error
        self.motor.configurator.apply(motor_config)

        ### Functionality Setup (velocity request)
        self.velocity_req = VelocityVoltage(0.0)

        # Logging
        FalconLogger.addLoggedObject("Agitator/Inputs/motor", self.motor)

    def periodic(self) -> None:
        # Logging: Write Current Measured Subsystem State

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled() or self.disabled:
            self.stop()
        else:
            self.run()
        
        # Logging: Write Post Operation Information
        FalconLogger.logOutput("/Agitator/Outputs/Setpoint", self.getDesiredSpeed())
        FalconLogger.logOutput("/Launcher/Outputs/isAtSpeed", self.isAtSpeed())

        FalconLogger.logOutput("systemStates/Agitator running", self.getDesiredSpeed() > 5)

    def run(self) -> None:
        # control velocity
        self.motor.set_control(self.velocity_req)

    def stop(self) -> None:
        self.velocity_req.velocity = 0.0
    
    def toggleDisabled(self) -> None:
        self.disabled = not self.disabled

    def setDesiredSpeed(self, speed:rotations_per_second) -> None:
        self.velocity_req.velocity = speed

    def getDesiredSpeed(self) -> rotations_per_second:
        return self.velocity_req.velocity
    
    def isAtSpeed(self) -> bool:
        return abs(self.motor.get_closed_loop_error().value) < self.Constants.kAtSpeedTolerance