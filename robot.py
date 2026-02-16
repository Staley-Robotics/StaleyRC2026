from phoenix6 import hardware, controls, configs, signals
from wpilib import TimedRobot, XboxController, Mechanism2d, MechanismRoot2d, MechanismLigament2d, SmartDashboard, Color8Bit, Color
from wpimath.system.plant import DCMotor
from wpilib.simulation import ElevatorSim;
from ntcore.util import ntproperty
from rev import SparkMax

class MyRobot(TimedRobot):
    intakeSpeed = ntproperty("/intakeSpeed", defaultValue=0.0, persistent=True)
    
    def __init__(self, period = 0.02):
        super().__init__(period)
        self.controller = XboxController(0)
        self.iMotor = hardware.TalonFX(0, "rio")
        # self.iMotor = SparkMax(3, SparkMax.MotorType.kBrushless) # hopefully the right motor id
        # self.iMotor.configs.apply(configs.TalonFXConfigs.defaultConfigs())

        self.config0 = configs.Slot0Configs()
        self.config0.k_p = 0.15
        self.config0.k_i = 0
        self.config0.k_d = 0

        self.iMotor.configurator.apply(self.config0)

        self.request = controls.PositionVoltage(self.iMotor.getEncoder().getPosition()).with_slot(0)

        self.mech = Mechanism2d(3, 3)
        self.root = self.mech.getRoot("root", 1, 0.5)
        self.arm = self.root.appendLigament("arm", 1, 90, 10, Color8Bit(255, 0, 0))

        self.elevatorSim = ElevatorSim(
            gearbox=DCMotor.krakenX60(1),
            gearRatio=10.0,
            drumRadius=0.05,
            minHeight=0.0,
            maxHeight=2.0,
            simulateGravity=False,
            startingHeight=0.0,
            maxHeight=2.0
        )

        SmartDashboard.putData("Climber Mech", self.mech)

    def teleopPeriodic(self):
        # trigger = self.controller.getRightTriggerAxis()
        increase = self.controller.getRightBumperButtonPressed()
        decrease = self.controller.getLeftBumperButtonPressed()
        if increase:
            self.intakeSpeed = min(1, max(self.intakeSpeed + 0.1, -1)) # +10 %
        if decrease:
            self.intakeSpeed = min(1, max(self.intakeSpeed - 0.1, -1)) # -10 %
        move = self.controller.getAButton()
        if move:
            self.iMotor.set(self.intakeSpeed)
        else:
            self.iMotor.set(0)