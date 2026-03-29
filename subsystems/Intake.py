from enum import Enum

from commands2 import Subsystem
from wpilib import RobotState, DutyCycleEncoder, SmartDashboard, RobotBase, RobotController, Mechanism2d, Color8Bit, Color
from wpilib.simulation import SingleJointedArmSim, LinearSystemSim_2_1_2
from wpimath.system.plant import DCMotor
from ntcore.util import ntproperty

from wpimath.units import *
from phoenix6.units import *

from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.configs import * #TalonFXConfiguration, MotorOutputConfigs, Slot0Configs, FeedbackConfigs, CANcoderConfiguration, MagnetSensorConfigs, ClosedLoopGeneralConfigs, Slot1Configs
from phoenix6.signals import InvertedValue, NeutralModeValue, FeedbackSensorSourceValue, GravityTypeValue, SensorDirectionValue, StaticFeedforwardSignValue, GainSchedBehaviorValue
from phoenix6.controls import PositionVoltage, VoltageOut
from phoenix6.sim import ChassisReference

from util.FalconLogger import FalconLogger

class IntakeConstants:
    kP:float=7.0    # proportion       The farther away, the harder it pushes
    kI:float=0.0    # integral         The longer it's been off, the harder it pushes
    kD:float=4.0    # differential     The harder it pushes, the less it pushes
    kS:float=0.4    # static
    kG:float=1.0    # gravity          Constant force, but accounting for gravity

    gear_ratio:float=11/60 # rotor/mechanism

    tolerance:degrees = 10

class Intake(Subsystem):
    class Speeds:
        STOP = 0
        IN = 0.30
        OUT = -0.4

    class Positions:
        '''
        Position setpoints for the intake in degrees
        0 should be the horizontal/outward/deployed position
        90 should be straight up
        '''
        MAX:degrees = (0.354004 * 360) - 5 # -5 degrees
        MIN:degrees = 1.5    # 0 (by definition)
        START:degree = MAX

        STORED:degrees =  MAX - 1
        INTAKING:degrees = MIN
        BOUNCE_UP:degrees = 70
    
    disablePivot = ntproperty("/Disabling/IntakePivot", False, persistent=False)

    def __init__(self, intakeMotorID:int, pivotMotorID:int, pivotEncoderID:int, pivotEncoderOffset:rotation) -> None:
        ### Motor Setup
        ## Intake Motor
        self.intake_motor = TalonFX(intakeMotorID, "rio")

        # Config
        intake_motor_config = TalonFXConfiguration()
        intake_motor_config = intake_motor_config.with_motor_output(
            MotorOutputConfigs()
            .with_neutral_mode(NeutralModeValue.COAST)
            .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
        )
        self.intake_motor.configurator.apply(intake_motor_config)

        ## Pivot Motor
        self.pivot_motor = TalonFX(pivotMotorID, "rio")

        # Config
        pivot_motor_config = TalonFXConfiguration()
        pivot_motor_config = pivot_motor_config.with_motor_output(
            MotorOutputConfigs()
            .with_neutral_mode(NeutralModeValue.BRAKE)
            .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
        ).with_slot0(
            Slot0Configs()
                .with_k_p(IntakeConstants.kP)
                .with_k_i(IntakeConstants.kI)
                .with_k_d(IntakeConstants.kD)
                .with_k_s(IntakeConstants.kS)
                .with_k_g(IntakeConstants.kG)
                .with_gravity_type(GravityTypeValue.ARM_COSINE)
                .with_gravity_arm_position_offset(-0.03)
                .with_static_feedforward_sign(StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN)
                .with_gain_sched_behavior(GainSchedBehaviorValue.USE_SLOT1)
        ).with_slot1(
            Slot1Configs()
                .with_k_p(0)#IntakeConstants.kP)
                .with_k_i(0)
                .with_k_d(0)#IntakeConstants.kD)
                .with_k_s(0)
                .with_k_g(0)
                .with_gravity_type(GravityTypeValue.ARM_COSINE)
                .with_gravity_arm_position_offset(-0.03)
                .with_static_feedforward_sign(StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN)
        ).with_feedback(
            FeedbackConfigs()
                .with_feedback_remote_sensor_id(pivotEncoderID)
                .with_feedback_sensor_source(FeedbackSensorSourceValue.REMOTE_CANCODER)
                .with_rotor_to_sensor_ratio(IntakeConstants.gear_ratio) #rotor tooth count / pivot tooth count
                .with_sensor_to_mechanism_ratio(1)
        ).with_closed_loop_general(
            ClosedLoopGeneralConfigs()
                .with_gain_sched_error_threshold(IntakeConstants.tolerance / 360)
        ).with_closed_loop_ramps(
            ClosedLoopRampsConfigs()
                .with_voltage_closed_loop_ramp_period(0.03)
        ).with_current_limits(
            CurrentLimitsConfigs()
            .with_stator_current_limit(120.0)
            .with_stator_current_limit_enable(True)
            .with_supply_current_limit(40)
            .with_supply_current_limit_enable(True)
            .with_supply_current_lower_limit(40)
            .with_supply_current_lower_time(1.0)
        )
        self.pivot_motor.configurator.apply(pivot_motor_config)

        # Encoder
        self.pivot_encoder = CANcoder(pivotEncoderID, "rio")
        encoder_cfg = CANcoderConfiguration()\
            .with_magnet_sensor(
                MagnetSensorConfigs()\
                .with_magnet_offset(pivotEncoderOffset)
                .with_sensor_direction(SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE)
            )
        #Apply
        self.pivot_encoder.configurator.apply(encoder_cfg)

        ### Functionality Setup
        self.intake_request = VoltageOut(0.0)
        self.pivot_request = PositionVoltage(self.getPivotPosition())
 
        ## Logging
        FalconLogger.addLoggedObject("/Intake/PivotMotor", self.pivot_motor)
        FalconLogger.addLoggedObject("/Intake/IntakeMotor", self.intake_motor)

        ## Mech2d
        mech = Mechanism2d( 100, 100, Color8Bit(50,50,70) )
        mechRoot = mech.getRoot( 'CoralPivot', 90, 10 )
        ligRobBase = mechRoot.appendLigament('robBase', 50, 180, color=Color8Bit( Color.kGray ) )
        # ligRobBase.appendLigament( 'test', 40, -131, color=Color8Bit(Color.kBlanchedAlmond), lineWidth=4 )
        self.mechArmTarget = ligRobBase.appendLigament( 'intakePivotTarget', 40, 0, color=Color8Bit(Color.kYellow), lineWidth=4 )
        self.mechArmActual = ligRobBase.appendLigament( 'intakePivotActual', 80, 0, color=Color8Bit(Color.kGreen) )
        if RobotBase.isSimulation(): self.mechArmSim = ligRobBase.appendLigament('intakePivotSSim', 60, 0, color=Color8Bit(Color.kRed) )

        SmartDashboard.putData('/Mechanisms/IntakePivot', mech)

        ### Simulation
        if RobotBase.isSimulation():

            self.pivot_motor_sim = self.pivot_motor.sim_state
            self.pivot_encoder_sim = self.pivot_encoder.sim_state

            self.pivot_motor_sim.set_motor_type(self.pivot_motor_sim.MotorType.KRAKEN_X60)
            self.pivot_motor_sim.orientation = ChassisReference.COUNTER_CLOCKWISE_POSITIVE
            self.pivot_encoder_sim.set_raw_position(degreesToRotations(self.Positions.START))

            self.arm_sim = SingleJointedArmSim(
                DCMotor.krakenX60(),
                IntakeConstants.gear_ratio,
                SingleJointedArmSim.estimateMOI( 0.3, 0.1 ),
                0.1,
                degreesToRadians( self.Positions.MIN ),
                degreesToRadians( self.Positions.MAX ),
                True, # Gravity
                degreesToRadians( self.Positions.START )
            )
            self.arm_sim.setState( degreesToRadians( self.getPivotPosition() ) , 0.0 )

    def periodic(self) -> None:
        # Logging: Write Current Measured Subsystem State
        FalconLogger.logInput("/Intake/Inputs/launchMotor/velocity", self.intake_motor.get_velocity().value)

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled():
            self.stop()
        else:
            self.run()

        # Mech2d
        self.mechArmActual.setAngle( -self.getPivotPosition() )
        self.mechArmTarget.setAngle( -self.getPivotSetpoint() )
        
        # Logging: Write Post Operation Information
        FalconLogger.logOutput("/Intake/Outputs/Setpoint (deg)", self.getPivotSetpoint())
        FalconLogger.logOutput("/Intake/Outputs/Error (deg)", self.pivot_motor.get_closed_loop_error().value * 360)
        FalconLogger.logOutput("/Intake/Outputs/closed loop reference (deg)", self.pivot_motor.get_closed_loop_reference().value * 360)
        FalconLogger.logOutput("/Intake/Outputs/Position (deg)", self.getPivotPosition())

        FalconLogger.logOutput("systemStates/Intake running", self.getIntakeSpeed() > 0.1)
        FalconLogger.logOutput("systemStates/Intake deployed", self.getPivotPosition() < 60)
    
    def simulationPeriodic(self):
        ## Simulation Physics
        # set the supply voltage of the TalonFX
        self.pivot_motor_sim.set_supply_voltage(RobotController.getBatteryVoltage())
        self.pivot_encoder_sim.set_supply_voltage(RobotController.getBatteryVoltage())

        # get the motor voltage of the TalonFX
        motor_voltage = self.pivot_motor_sim.motor_voltage
        FalconLogger.logOutput('/Intake/sim motor voltage', motor_voltage)


        # use the motor voltage to calculate new position and velocity
        # using WPILib's DCMotorSim class for physics simulation
        self.arm_sim.setInputVoltage(motor_voltage)
        self.arm_sim.update(0.020) # assume 20 ms loop time

        # apply the new rotor position and velocity to the TalonFX;
        # note that this is rotor position/velocity (before gear ratio), but
        # DCMotorSim returns mechanism position/velocity (after gear ratio)
        self.pivot_motor_sim.set_raw_rotor_position(
            IntakeConstants.gear_ratio
            * radiansToRotations(self.arm_sim.getAngle())
        )
        self.pivot_motor_sim.set_rotor_velocity(
            IntakeConstants.gear_ratio
            * radiansToRotations(self.arm_sim.getVelocity())
        )
        self.pivot_encoder_sim.set_raw_position(
            radiansToRotations(self.arm_sim.getVelocity())
        )
        self.pivot_encoder_sim.set_velocity(
            radiansToRotations(self.arm_sim.getVelocity())
        )

        ## Logging and stuff
        self.mechArmSim.setAngle( radiansToDegrees( -self.arm_sim.getAngle() ) )

    def run(self) -> None:
        ## Intake
        #control speed by percentage
        self.intake_motor.set_control(self.intake_request)

        ## Pivot
        #control position
        if not self.disablePivot:
            if abs(self.pivot_request.position * 360 - self.getPivotPosition()) < IntakeConstants.tolerance:
                self.pivot_motor.set_control(VoltageOut(0.0))
            else:
                self.pivot_motor.set_control(self.pivot_request)
        else:
            self.pivot_motor.set_control(VoltageOut(0.0))
    
    def toggleDisabled(self) -> None:
        self.disablePivot = not self.disablePivot

    def stop(self) -> None:
        self.setIntakeSpeed(self.Speeds.STOP)
        self.setPivotSetpoint(self.getPivotPosition())

    def setIntakeSpeed(self, speed:Speeds|percent) -> None:
        """speed: as percentage (-1 to 1) or Intake.Speeds constant"""
        self.intake_request.output = speed * 12

    def getIntakeSpeed(self) -> percent:
        return self.intake_request.output / 12
    
    def setPivotSetpoint(self, setpoint:Positions|degrees) -> None:
        self.pivot_request.position = min(max(setpoint, self.Positions.MIN), self.Positions.MAX) / 360 # deg to rot
    
    def getPivotSetpoint(self) -> degrees:
        return self.pivot_request.position * 360
    
    def getPivotPosition(self) -> degrees:
        return self.pivot_encoder.get_absolute_position().value * 360 # rot to deg
    
    def getAtSetpoint(self) -> bool:
        return abs(self.pivot_motor.get_closed_loop_error().value * 360) < IntakeConstants.tolerance