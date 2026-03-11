from enum import Enum

from commands2 import Subsystem
from wpilib import RobotState, DutyCycleEncoder, SmartDashboard
from wpimath.controller import PIDController, ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians
from wpilib.simulation import SingleJointedArmSim, LinearSystemSim_2_1_2
from wpimath.system.plant import DCMotor
from ntcore.util import ntproperty

from wpimath.units import *
from phoenix6.units import *

from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.configs import TalonFXConfiguration, MotorOutputConfigs, Slot0Configs, FeedbackConfigs, CANcoderConfiguration, MagnetSensorConfigs
from phoenix6.signals import InvertedValue, NeutralModeValue, FeedbackSensorSourceValue, GravityTypeValue, SensorDirectionValue
from phoenix6.controls import PositionVoltage, VoltageOut

# from rev

from util.FalconLogger import FalconLogger

class IntakeConstants:
    kP:float=0.0 # proportion       The farther away, the harder it pushes
    kI:float=0.0 # integral         The longer it's been off, the harder it pushes
    kD:float=0.0 # differential     The harder it pushes, the less it pushes
    kG:float=0.0 # gravity          Constant force, but accounting for gravity

    gear_ratio:float=16 #total guess

class Intake(Subsystem):
    class IntakeSpeeds:
        STOP = 0
        IN = 0.5

    class IntakePositions:
        '''
        Position setpoints for the intake in degrees
        0 is (should be) the horizontal/outward/deployed position
        90 is (should be) straight up
        '''
        MAX:degrees = 131.4 # 0.403809 rot measured -(arbitrarily)-> 0.365 rot for safety
        MIN:degrees = 10    # 0 (by definition)
        START:degree = MAX

        STORED:degrees =  120
        INTAKING:degrees = 5
        BOUNCE_UP:degrees = 30

    def __init__(self, intakeMotorID:int, pivotMotorID:int, pivotEncoderID:int, pivotEncoderOffset:rotation) -> None:
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
        # self.pivot_encoder = DutyCycleEncoder( pivotEncoderID )
        # self.controller = PIDController(
        #     IntakeConstants.kP,
        #     IntakeConstants.kI,
        #     IntakeConstants.kD,
        # )
        # self.controller = ProfiledPIDControllerRadians(
        #     IntakeConstants.kP,
        #     IntakeConstants.kI,
        #     IntakeConstants.kD,   
        #     TrapezoidProfileRadians.Constraints(

        #     )
        # )
        # SmartDashboard.putData("/Intake/PID", self.controller)

        # Config
        pivot_motor_config = TalonFXConfiguration()
        pivot_motor_config = pivot_motor_config.with_motor_output(
            MotorOutputConfigs()
            .with_neutral_mode(NeutralModeValue.COAST)
            .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
        ).with_slot0(
            Slot0Configs()
                .with_k_p(IntakeConstants.kP)
                .with_k_i(IntakeConstants.kI)
                .with_k_d(IntakeConstants.kD)
                .with_k_g(IntakeConstants.kG)
                .with_gravity_type(GravityTypeValue.ARM_COSINE)
                .with_gravity_arm_position_offset(0.0)
        ).with_feedback(
            FeedbackConfigs()
                .with_feedback_remote_sensor_id(pivotEncoderID)
                .with_feedback_sensor_source(FeedbackSensorSourceValue.REMOTE_CANCODER)
                .with_rotor_to_sensor_ratio(11/60) #rotor tooth count / pivot tooth count?
                .with_sensor_to_mechanism_ratio(1)
        )
        # pivot_motor_config = pivot_motor_config.with_closed_loop_general(
        # )
        self.pivot_motor.configurator.apply(pivot_motor_config)

        # Encoder
        #just for configs - accessed thru motor
        self.pivot_encoder = CANcoder(pivotEncoderID, "rio")
        encoder_cfg = CANcoderConfiguration()\
            .with_magnet_sensor(
                MagnetSensorConfigs()\
                .with_magnet_offset(pivotEncoderOffset)
                .with_sensor_direction(SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE)
            )
        #Apply
        self.pivot_encoder.configurator.apply(encoder_cfg)
        self.pivot_encoder.set_position(0)

        ### Functionality Setup
        self.intake_request = VoltageOut(0.0)
        self.pivot_request = PositionVoltage(self.getPivotPosition())

        ## Logging
        FalconLogger.addLoggedObject("/Intake/PivotMotor", self.pivot_motor)
        FalconLogger.addLoggedObject("/Intake/IntakeMotor", self.intake_motor)

        ### Simulation
        # self.arm_sim = SingleJointedArmSim(
        #     DCMotor.krakenX60(),
        #     IntakeConstants.gear_ratio,
        #     SingleJointedArmSim.estimateMOI( 0.0, 0.0 ),
        #     0.0,
        #     degreesToRadians( self.IntakePositions.MIN ),
        #     degreesToRadians( self.IntakePositions.MAX ),
        #     True, # Gravity
        #     degreesToRadians( self.IntakePositions.START )
        # )

    def periodic(self) -> None:
        # Logging: Write Current Measured Subsystem State
        FalconLogger.logInput("/Intake/Inputs/launchMotor/velocity", self.intake_motor.get_velocity().value)

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled():
            self.stop()
        else:
            self.run()
        
        # Logging: Write Post Operation Information
        FalconLogger.logOutput("/Intake/Outputs/Setpoint", self.getPivotSetpoint())
        FalconLogger.logOutput("/Intake/Outputs/Error", self.pivot_motor.get_closed_loop_error().value)
        FalconLogger.logOutput("/Intake/Outputs/closed loop reference * 360", self.pivot_motor.get_closed_loop_reference().value * 360)
        FalconLogger.logOutput("/Intake/Outputs/Position", self.getPivotPosition())

    def run(self) -> None:
        ## Intake
        #control speed by percentage
        self.intake_motor.set_control(self.intake_request)

        ## Pivot
        #control position
        # calc = self.controller.calculate(self.getPivotPosition(), self.getPivotSetpoint()) # pid calc
        # calc += cos(self.getPivotPosition()) # Rotational FF
        self.pivot_motor.set_control(self.pivot_request)

    def stop(self) -> None:
        self.setIntakeSpeed(self.IntakeSpeeds.STOP)
        self.setPivotSetpoint(self.getPivotPosition())

    def setIntakeSpeed(self, speed:IntakeSpeeds|percent) -> None:
        self.intake_request.output = speed * 12

    def getIntakeSpeed(self) -> percent:
        return self.intake_request.output / 12
    
    def setPivotSetpoint(self, setpoint:IntakePositions|degrees) -> None:
        self.pivot_request.position = min(max(setpoint, self.IntakePositions.MIN), self.IntakePositions.MAX) / 360 # deg to rot
    
    def getPivotSetpoint(self) -> degrees:
        return self.pivot_request.position * 360
    
    def getPivotPosition(self) -> degrees:
        return self.pivot_encoder.get_absolute_position().value * 360 # rot to deg