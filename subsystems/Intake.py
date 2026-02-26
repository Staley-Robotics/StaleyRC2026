from enum import Enum

from commands2 import Subsystem
from wpilib import RobotState, DutyCycleEncoder
from wpimath.controller import PIDController
from ntcore.util import ntproperty

from wpimath.units import *
from phoenix6.units import *

from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration, MotorOutputConfigs, Slot0Configs, FeedbackConfigs
from phoenix6.signals import InvertedValue, NeutralModeValue, FeedbackSensorSourceValue
from phoenix6.controls import PositionVoltage

# from rev

from util.FalconLogger import FalconLogger

class IntakeConstants:
    kP:float=0.0 # proportion       The farther away, the harder it pushes
    kI:float=0.0 # integral         The longer it's been off, the harder it pushes
    kD:float=0.0 # differential     The harder it pushes, the less it pushes
    kG:float=0.0 # gravity          Constant force, but accounting for gravity

class Intake(Subsystem):
    class IntakeSpeeds:
        STOP = 0
        IN = ntproperty("/Settings/Intake/IntakeSpeed", defaultValue=0.8, persistent=True)

    class IntakePositions:
        '''
        Position setpoints for the intake in degrees
        0 is (should be) the horizontal/outward/deployed position
        90 is (should be) straight up
        '''
        MAX:degrees = 80 #NOTE: currently underestimate for safety in testing
        MIN:degrees = 10 #NOTE: currently underestimate for safety in testing

        IN:degrees = 80 #NOTE: currently underestimate for safety in testing
        OUT:degrees = 10 #NOTE: currently underestimate for safety in testing

    def __init__(self, intakeMotorID:int, pivotMotorID:int, pivotEncoderID:int) -> None:
        ### Motor Setup
        ## Intake Motor
        self.intake_motor = TalonFX(intakeMotorID, "rio")

        # Config
        intake_motor_config = TalonFXConfiguration()
        intake_motor_config = intake_motor_config.with_motor_output(
            MotorOutputConfigs()
            .with_neutral_mode(NeutralModeValue.BRAKE)
            .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
        )
        self.intake_motor.configurator.apply(intake_motor_config)

        ## Pivot Motor
        self.pivot_motor = TalonFX(pivotMotorID, "rio")
        # Encoder
        self.pivot_encoder = DutyCycleEncoder
        self.comtroller = PIDController(
            IntakeConstants.kP,
            IntakeConstants.kI,
            IntakeConstants.kD,
            IntakeConstants.kG,
        )

        # Config
        pivot_motor_config = TalonFXConfiguration()
        pivot_motor_config = pivot_motor_config.with_motor_output(
            MotorOutputConfigs()
            .with_neutral_mode(NeutralModeValue.COAST))
        # ).with_slot0(
        #     Slot0Configs()
        #         .with_k_p(IntakeConstants.kP)
        #         .with_k_i(IntakeConstants.kI)
        #         .with_k_d(IntakeConstants.kD)
        #         .with_k_g(IntakeConstants.kG)
        # ).with_feedback(
        #     FeedbackConfigs()
        #         .with_feedback_remote_sensor_id(pivotEncoderID)
        #         .with_feedback_sensor_source(FeedbackSensorSourceValue.REMOTE_CANCODER)
        # )
        pivot_motor_config = pivot_motor_config.with_closed_loop_general(
        )
        self.pivot_motor.configurator.apply(pivot_motor_config)

        ### Functionality Setup
        self.intake_speed = self.IntakeSpeeds.STOP
        self.pivot_setpoint = self.IntakePositions.IN

    def periodic(self) -> None:
        # Logging: Write Current Measured Subsystem State
        FalconLogger.logInput("/Intake/Inputs/launchMotor/velocity", self.intake_motor.get_velocity())

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled():
            self.stop()
        else:
            self.run()
        
        # Logging: Write Post Operation Information
        FalconLogger.logOutput("/Intake/Outputs/Setpoint", self.getSetpoint())

    def run(self) -> None:
        ## Intake
        #control speed by percentage
        self.intake_motor.set(self.intake_speed)

        ## Pivot
        #control position
        self.pivot_motor.set_control(PositionVoltage(self.pivot_setpoint))

    def stop(self) -> None:
        self.intake_speed = 0
        self.pivot_setpoint = self.getPivotPosition()

    def setIntakeSpeed(self, speed:IntakeSpeeds) -> None:
        self.intake_speed = speed

    def getIntakeSpeed(self) -> IntakeSpeeds:
        return self.intake_speed
    
    def setPivotSetpoint(self, setpoint:IntakePositions|degrees) -> None:
        self.pivot_setpoint = setpoint
    
    def getPivotSetpoint(self) -> degrees:
        return self.pivot_setpoint
    
    def getPivotPosition(self) -> rotation:
        return self.pivot_motor.get_position().value