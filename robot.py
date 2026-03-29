# Python Imports
from pathlib import Path

# FRC Imports
from wpilib import DriverStation, DataLogManager, RobotBase, TimedRobot, XboxController
from commands2 import Command, CommandScheduler
from rev import SparkMax
from phoenix6.signal_logger import SignalLogger
from ntcore.util import ntproperty

# Local Imports
from RobotContainer import RobotContainer
from util import FalconLogger

class MyRobot(TimedRobot):
    # Variable Declaration
    __robotContainer:RobotContainer = None
    __autoCmd:Command = None
    __logger:FalconLogger = None

    climberSpeed = ntproperty("/climberSpeed", defaultValue=0.0, persistent=True)

    controller = XboxController(0)

    # Initialization
    def robotInit(self):
        # Disable Joystick Notifications
        DriverStation.silenceJoystickConnectionWarning(True)

        # Start Logging using the built in DataLogManager
        logDir = '/U/logs' if RobotBase.isReal() else '.logs'
        DataLogManager.start( dir=(logDir if Path(logDir).is_dir() else ''), period=1.0 )
        DriverStation.startDataLog( DataLogManager.getLog() )

        # handle phoenix logs
        if RobotBase.isSimulation() or not RobotBase.isReal():
            SignalLogger.set_path('.logs/ctre')
        
        # Built The Robot
        self.__robotContainer = RobotContainer()
        self.__logger = FalconLogger(False)

        # and a test climb motor
        # self.iMotor = SparkMax(14, SparkMax.MotorType.kBrushless)

    # Periodic Loop / All Modes
    def robotPeriodic(self):
        # Mark the Current Timestamp for Logging
        self.__logger.setTime()

        # Run the CommandScheduler Loop
        CommandScheduler.getInstance().run()

        # Write the Log Results
        try:
            self.__logger.writeLog()
        except Exception as err:
            print(f"WARNING! FalconLogger Cannot Write to Log!: {err}")

        self.__robotContainer.gameCalc.debugLog()

    # Autonomous Mode
    def autonomousInit(self):
        # Start the Autonomous Package
        self.autonomousCommand = self.__robotContainer.getAutonomousCommand()

        if self.autonomousCommand:
            CommandScheduler.getInstance().schedule(self.autonomousCommand)
    
    def autonomousPeriodic(self): pass

    def autonomousExit(self):
        # End the Autonomous Package
        try:
            self.__autoCmd.cancel()
        except:
            pass

    # Teleop Mode
    def teleopInit(self): pass
    def teleopPeriodic(self): pass# stuff here for climber
        # Buttons
        # a_pressed = self.controller.getAButtonPressed()
        # b_pressed = self.controller.getBButtonPressed()
        # back_pressed = self.controller.getBackButtonPressed()
        # l_bumper_pressed = self.controller.getLeftBumperButtonPressed()
        # r_bumper_pressed = self.controller.getRightBumperButtonPressed()

        # if (r_bumper_pressed):
        #     self.climberSpeed = min(1, max(self.climberSpeed + 0.1, -1)) # 
        # if (l_bumper_pressed):
        #     self.climberSpeed = min(1, max(self.climberSpeed - 0.1, -1)) #

        # self.iMotor.set(self.climberSpeed)

    def teleopExit(self): pass

    # Test Mode
    def testInit(self): pass
    def testPeriodic(self): pass
    def testExit(self): pass

    # Disable Mode
    def disabledInit(self): pass
    def disabledPeriodic(self): pass
    def disabledExit(self): pass

    # Simulation Mode
    def _simulationInit(self): pass
    def _simulationPeriodic(self): pass
    def _simulationExit(self): pass