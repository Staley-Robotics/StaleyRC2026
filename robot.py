import math

from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import PositionVoltage
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue
from wpilib import TimedRobot, XboxController, Mechanism2d, MechanismRoot2d, MechanismLigament2d, SmartDashboard, Color8Bit, Color
import wpilib
from wpimath.system.plant import DCMotor
from wpilib.simulation import ElevatorSim, SingleJointedArmSim;
from ntcore.util import ntproperty
from rev import SparkMax

class MyRobot(TimedRobot):
    stepSizes = [10.0, 1.0, 0.1, 0.01, 0.001]
    stepIndex = 2

    intakeSpeed = ntproperty("/intakeSpeed", defaultValue=0.0, persistent=True)
    stepSize = ntproperty("/stepSize", defaultValue=stepSizes[stepIndex], persistent=True)
    # Pivot geometry / conversion
    GEAR_RATIO = 60/11 # motor rotations per 1 pivot rotation
    LOW_ANGLE_DEG = 20.0
    HIGH_ANGLE_DEG = 75.0
    MIN_ANGLE_DEG = 0.0
    MAX_ANGLE_DEG = 90.0

    # Approximate mechanism properties (realistic defaults)
    ARM_LENGTH_M = 0.33
    ARM_MASS_KG = 4.0 # actually pretty close to the real weight

    SIM_KP_VOLTS_PER_DEG = .20 # simple proportional voltage for simulation, to be tuned

    
    def __init__(self, period = 0.02):
        super().__init__(period)
        self.controller = XboxController(0)
        # self.pivotMotor = TalonFX(0, "rio")
        # self.position_request = PositionVoltage(0.0)
        self.iMotor = SparkMax(14, SparkMax.MotorType.kBrushless) # hopefully the right motor id
        # self.iMotor.configs.apply(configs.TalonFXConfigs.defaultConfigs())
        # self.intakeSpeed = ntproperty("/intakeSpeed", defaultValue=0.0, persistent=True);
    

        # self.config0 = configs.Slot0Configs()
        # self.config0.k_p = 0.15
        # self.config0.k_i = 0
        # self.config0.k_d = 0

        # self.iMotor.configurator.apply(self.config0)

        # self.config = TalonFXConfiguration()
        # self.config.motor_output.neutral_mode = NeutralModeValue.BRAKE # may change to FLOAT
        # # Closed-loop PID gains for PositionVoltage (Slot0)
        # self.config.slot0.k_p = 1.0 # to be tuned
        # self.config.slot0.k_i = 0.0 # to be tuned
        # self.config.slot0.k_d = 0.25 # to be tuned
        # self.config.slot0.k_g = 0.0 # to be tuned

        # Keep voltage constrained during testing
        # self.config.voltage.peak_forward_voltage = 8.0
        # self.config.voltage.peak_reverse_voltage = -8.0
        
        # # apply config
        # self.pivotMotor.configurator.apply(self.config)

        # set target
        # self.target_angle_deg = self.LOW_ANGLE_DEG
        # self.target_motor_rot = self._angle_deg_to_motor_rot(self.target_angle_deg)

        # self.request = PositionVoltage(self.iMotor.getEncoder().getPosition()).with_slot(0)

        # # Mechanism2d setup for simulation visualization
        # self.mech = Mechanism2d(3.0, 3.0, Color8Bit(20, 20, 30))
        # self.root = self.mech.getRoot("root", 1, 0.5)
        # self.arm = self.root.appendLigament("arm", 1, 90, 10, Color8Bit(255, 0, 0))
        # self.base_ligament = self.root.appendLigament(
        #     "Base",
        #     0.5,
        #     90.0,
        #     5,
        #     color=Color8Bit(Color.kGray),
        # )
        # self.pivot_target_ligament = self.base_ligament.appendLigament(
        #     "PivotTarget",
        #     1.1,
        #     self.target_angle_deg,
        #     3,
        #     color=Color8Bit(Color.kYellow),
        # )
        # self.pivot_actual_ligament = self.base_ligament.appendLigament(
        #     "PivotActual",
        #     1.0,
        #     self.target_angle_deg,
        #     6,
        #     color=Color8Bit(Color.kGreen),
        # )

        # self.elevatorSim = ElevatorSim(
        #     gearbox=DCMotor.krakenX60(1),
        #     gearRatio=10.0,
        #     drumRadius=0.05,
        #     minHeight=0.0,
        #     maxHeight=2.0,
        #     simulateGravity=False,
        #     startingHeight=0.0,
        #     maxHeight=2.0
        # )

        # self.arm_sim = SingleJointedArmSim(
        #     DCMotor.krakenX60(1),
        #     self.GEAR_RATIO,
        #     SingleJointedArmSim.estimateMOI(self.ARM_LENGTH_M, self.ARM_MASS_KG),
        #     self.ARM_LENGTH_M,
        #     math.radians(self.MIN_ANGLE_DEG),
        #     math.radians(self.MAX_ANGLE_DEG),
        #     True,
        #     math.radians(self.LOW_ANGLE_DEG)
        # )

        # SmartDashboard.putData("Climber Mech", self.mech)
        # SmartDashboard.putData("PivotMechanism", self.mech)
        SmartDashboard.putString("RobotStatus", "Pivot test robot initialized")
        SmartDashboard.putNumber("Pivot/LowAngleDeg", self.LOW_ANGLE_DEG)
        SmartDashboard.putNumber("Pivot/HighAngleDeg", self.HIGH_ANGLE_DEG)

    def teleopInit(self) -> None:
        SmartDashboard.putString("Mode", "Teleop")

    def teleopPeriodic(self):
        # Buttons
        a_pressed = self.controller.getAButtonPressed()
        b_pressed = self.controller.getBButtonPressed()
        # dpad_up = self.controller.getPOV() > 340 and self.controller.getPOV() < 20 # 20 degree threshold
        # dpad_down = self.controller.getPOV() > 160 and self.controller.getPOV() < 200 # 20 degree threshold
        # dpad_up_pressed = dpad_up and not #logic tbd a;lkfj as;ldfj 

        if (a_pressed):
            self.intakeSpeed = min(1, max(self.intakeSpeed + 0.1, -1)) # 
        if (b_pressed):
            self.intakeSpeed = min(1, max(self.intakeSpeed - 0.1, -1)) #

        self.iMotor.set(self.intakeSpeed)

        # Change target angle
        # if a_pressed: # increase angle
        #     self.target_angle_deg = min(self.target_angle_deg + self.stepSize, self.HIGH_ANGLE_DEG)
        # if b_pressed: # decrease angle
        #     self.target_angle_deg = max(self.target_angle_deg - self.stepSize, self.LOW_ANGLE_DEG)

        

        # Set target motor rot based on target angle
        # self.target_motor_rot = self._angle_deg_to_motor_rot(self.target_angle_deg)

        # self.pivotMotor.set_control(
        #     self.position_request.with_position(self.target_motor_rot)
        # )
        
        # Update telemetry from hardware position
        # motor_rot = self.pivotMotor.get_position().value_as_double
        # measured_angle_deg = self._motor_rot_to_angle_deg(motor_rot)

        # SmartDashboard.putNumber("Pivot/TargetAngleDeg", self.target_angle_deg)
        # SmartDashboard.putNumber("Pivot/MeasuredAngleDeg", measured_angle_deg)
        # SmartDashboard.putNumber("Pivot/TargetMotorRot", self.target_motor_rot)
        # SmartDashboard.putNumber("Pivot/MotorRot", motor_rot)


    def _simulationPeriodic(self):
        pass
        # Drive arm simulation toward target angle with a simple proportional voltage.
        # error_deg = self.target_angle_deg - self.arm_sim.getAngleDegrees()
        # applied_voltage = max(-12.0, min(12.0, error_deg * self.SIM_KP_VOLTS_PER_DEG))
        # self.arm_sim.setInputVoltage(applied_voltage)
        # self.arm_sim.update(self.getPeriod())

        # sim_angle_deg = self.arm_sim.getAngleDegrees()

        # Update mechanism drawing
        # self.pivot_target_ligament.setAngle(self.target_angle_deg)
        # self.pivot_actual_ligament.setAngle(sim_angle_deg)

        # SmartDashboard.putNumber("Pivot/SimAngleDeg", sim_angle_deg)
        # SmartDashboard.putNumber("Pivot/SimVelocityDps", self.arm_sim.getVelocityDps())
        # SmartDashboard.putNumber("Pivot/SimCurrentDrawA", self.arm_sim.getCurrentDraw())
        # SmartDashboard.putNumber("Pivot/SimAppliedVoltage", applied_voltage)

    # def teleopPeriodic(self):
    #     # trigger = self.controller.getRightTriggerAxis()
    #     increase = self.controller.getRightBumperButtonPressed()
    #     decrease = self.controller.getLeftBumperButtonPressed()
    #     if increase:
    #         self.intakeSpeed = min(1, max(self.intakeSpeed + 0.1, -1)) # +10 %
    #     if decrease:
    #         self.intakeSpeed = min(1, max(self.intakeSpeed - 0.1, -1)) # -10 %
    #     move = self.controller.getAButton()
    #     if move:
    #         self.iMotor.set(self.intakeSpeed)
    #     else:
    #         self.iMotor.set(0)

    @classmethod
    def _angle_deg_to_motor_rot(cls, angle_deg: float) -> float:
        return (angle_deg / 360.0) * cls.GEAR_RATIO

    @classmethod
    def _motor_rot_to_angle_deg(cls, motor_rot: float) -> float:
        return (motor_rot / cls.GEAR_RATIO) * 360.0


if __name__ == "__main__":
    robot = MyRobot()
    wpilib.run(robot)