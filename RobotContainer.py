# FRC Imports
from wpilib import SendableChooser, SmartDashboard
from commands2 import Command, cmd

# Local Imports
from subsystems import SampleSubsystem, Climber
from commands import SampleCommand, ClimberDecreasePos, ClimberIncreasePos, ClimberUpdatePos
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
        sysSample = SampleSubsystem( 0 ) 
        climber = Climber(0)

        # Commands
        climberIncPos = ClimberIncreasePos( climber )
        climberDecPos = ClimberDecreasePos( climber )
        climberUpdatePos = ClimberUpdatePos( climber )


        # Autonomous Chooser
        self.__autoChooser.setDefaultOption( "1 - None", cmd.none() )
        SmartDashboard.putData( "Autonomous Mode", self.__autoChooser )

        # Default Commands
       # sysSample.setDefaultCommand( cmdSampleLeft )

        # Driver Controller Button Binding
        driver1.y().onTrue( climberIncPos )
        driver1.a().onTrue( climberDecPos )
        driver1.b().whileTrue( climberUpdatePos )

    # Get Autonomous Command
    def getAutonomousCommand(self) -> Command:
        """
        Get the Autonomous Command that is currently selected in the AutoChooser Dropdown on the Shuffleboard / SmartDashboards
        """
        chooserValue = self.__autoChooser.getSelected()
        return chooserValue if isinstance( chooserValue, Command ) else cmd.none()
