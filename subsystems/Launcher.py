from wpilib import XboxController
from wpimath.controller import SimpleMotorFeedforwardMeters, PIDController
from commands2 import subsystem

from ntcore.util import ntproperty 

from phoenix6.hardware import TalonFX # Motor controller class for Falcons & Krakens

class Launcher(subsystem):
  
    class Speeds:
        STOP: 0
        SLOW: float = -0.1
        FULL: float = -33

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
    FeedForward = SimpleMotorFeedforwardMeters(.2, .1, 6.16)

    # adds "motor1 speed mult" as an editable property on the networktables
    # this allows for the value to be adjusted without redeploying through programs like Glass or Shuffleboard

    def __init__(self, motor_port:int):
        self.flywheel = TalonFX(motor_port)

        
    def periodic(self):
        '''
        Runs every frame while the robot is enabled and in Teleop
        '''
        self.Current_V = self.flywheel.get_velocity().value

        self.PIDOut = self.PID.calculate(self.Current_V, self.target_V)

        self.FeedForwardOut = self.FeedForward.calculate(self.target_V)

        self.flywheel.setVoltage(self.PIDOut + self.FeedForwardOut)


    def stop(self):
        self.target_V = self.Speeds.STOP


    def setSpeed(self, speed:float):

        self.target_V = speed

