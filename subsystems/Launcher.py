from wpilib import XboxController
from wpimath.controller import SimpleMotorFeedforwardRadians, PIDController
from commands2 import subsystem

from ntcore.util import ntproperty 

from phoenix6.hardware import TalonFX # Motor controller class for Falcons & Krakens
from phoenix6.configs import TalonFXConfiguration, MotorOutputConfigs, Slot0Configs
from phoenix6.controls import VelocityVoltage

class Launcher(subsystem):
  
    class Speeds:
        STOP: 0.0
        SLOW: float = -0.1
        FULL: float = -33.0

    class LauncherConstraints:
        kP = 0.2
        kI = 0.0
        kD = 0.0
        kV = 0.2
        kS = 0.1
        kA = 6.16

    flywheel_speed = ntproperty("/motor2 speed", 1.0)
    max_Velocity = ntproperty("Target Velocity", 1.0) #best one is -33
    flywheel_speed_mult = 0

    PID = PIDController(.2, 0, 0)
    FeedForward = SimpleMotorFeedforwardRadians(.2, .1, 6.16)

    # adds "motor1 speed mult" as an editable property on the networktables
    # this allows for the value to be adjusted without redeploying through programs like Glass or Shuffleboard

    def __init__(self, motor_port:int):
        self.flywheel = TalonFX(motor_port)

        self.config = TalonFXConfiguration()
        self.flywheelConfig = self.config.with_motor_output(MotorOutputConfigs).with_slot0(
            Slot0Configs()
                .with_k_p(self.LauncherConstraints.kP)
                .with_k_i(self.LauncherConstraints.kI)
                .with_k_d(self.LauncherConstraints.kD)
                .with_k_v(self.LauncherConstraints.kV)
                .with_k_s(self.LauncherConstraints.kS)
                .with_k_a(self.LauncherConstraints.kA)
        )

        self.flywheel.configurator.apply(self.flywheelConfig)

        self.request = VelocityVoltage(0).with_slot(0)

        
    def run(self):
        self.flywheel.set_control(self.request)


    def stop(self):
        self.request.velocity = self.Speeds.STOP


    def setSpeed(self, speed:float):

        self.request.velocity = speed

