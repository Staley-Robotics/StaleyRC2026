from enum import Enum

from commands2 import Subsystem
from wpilib import RobotState
from ntcore.util import ntproperty

from wpimath.units import *
from phoenix6.units import *

from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration, MotorOutputConfigs, Slot0Configs, ClosedLoopGeneralConfigs
from phoenix6.signals import InvertedValue, NeutralModeValue
from phoenix6.controls import VelocityVoltage

from util.FalconLogger import FalconLogger

class Launcher(Subsystem):
    '''This is functionally quite similar (if not the same) as Agitator, but for now they are kept seperate for simplicity's sake'''
    class LauncherSpeeds:
        WAIT:rotations_per_second = 5 # default speed for lower power consumption but faster acceleration when needed
        SPEED_AT_ZERO_DIST: 20 # total guess, speed at minimum distance TODO: measure
        SPEED_AT__DIST: 70 # total guess, speed at some arbitrary larger distance TODO: measure

    class Constants:
        # PID constants were tuned with a setSpeed of 30 rots/sec
        """
        Extra notes:
        kV was around .1, but as kP was increased to decrease time from rest to targetSpeed,
        kV was decreased to prevent overshoot. This is because the controller is using a velocity feedforward,
        so if kP is too low, it will rely on the feedforward to get to speed, but if kP is high enough,
        it will rely more on feedback to get to speed,
        so the feedforward needs to be less aggressive to prevent overshooting.
        """
        k_P:float=0.2
        k_I:float=0.0
        k_D:float=0.005
        k_S:float=0.25
        k_V:float=0.07

        kAtSpeedTolerance:rotations_per_second = 3.0 # ntproperty("/Launcher/At speed tolerance (rps)", 2.0, persistent=True) #total guess

        '''
        kraken free speed max: 6000 rpm = 100 rps
        cut to 70 for safety (and because free speed is gonna be higher than max in our mechanism)

        NOTE: actual flywheel speed will be double the motor speed because of gearing
        '''
        kMaxAllowedSpeed:rotations_per_second = 70.0
        kSpeedAt12Volts:rotations_per_second = 83.0


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
        if self.getDesiredSpeed() == 0:
            self.motor.set(0) # allows to coast down to 0 vel rather than hard stopping
        else:
            self.motor.set_control(self.velocity_req)

    def stop(self) -> None:
        self.velocity_req.velocity = 0.0

    def setDesiredSpeed(self, speed:rotations_per_second) -> None:
        self.velocity_req.velocity = speed

    def getDesiredSpeed(self) -> rotations_per_second:
        return self.velocity_req.velocity
    
    def isAtSpeed(self) -> bool:
        return abs(self.motor.get_closed_loop_error().value) < self.Constants.kAtSpeedTolerance