# FRC Imports
from wpilib import SendableChooser, SmartDashboard
from commands2 import Command, cmd

# Local Imports
from subsystems import Climber
from commands import ClimberChangePos
from util import FalconXboxController

class RobotContainer:
    """
    RobotContainer is the Initial Container for an FRC Robot
    """
    # Variable Declaration
    __autoChooser:SendableChooser = SendableChooser()

    # Initialization
    def __init__(self):
        """
        Initializes RobotContainer
        """
        # Driver Controller
        driver1 = FalconXboxController( 0 )

        # Declare Subsystems
        climber = Climber(0)

        # Commands
        climberIncPos = ClimberChangePos( climber, 1 ) #ClimberIncreasePos( climber )
        climberDecPos = ClimberChangePos( climber, -1 ) #ClimberDecreasePos( climber )



        # Autonomous Chooser
        self.__autoChooser.setDefaultOption( "1 - None", cmd.none() )
        SmartDashboard.putData( "Autonomous Mode", self.__autoChooser )

        # Default Commands
       # sysSample.setDefaultCommand( cmdSampleLeft )

        # Driver Controller Button Binding
        driver1.y().whileTrue( climberIncPos )
        driver1.a().whileTrue( climberDecPos )

    # Get Autonomous Command
    def getAutonomousCommand(self) -> Command:
        """
        Get the Autonomous Command that is currently selected in the AutoChooser Dropdown on the Shuffleboard / SmartDashboards
        """
        chooserValue = self.__autoChooser.getSelected()
        return chooserValue if isinstance( chooserValue, Command ) else cmd.none()
