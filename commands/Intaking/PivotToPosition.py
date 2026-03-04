import typing

from commands2 import Command, Subsystem
from wpimath.units import degrees
from ntcore.util import ntproperty

from subsystems import Intake

class PivotToPosition(Command):
    # Variable Declaration
    intake_sys:Intake = None
    
    # Initialization
    def __init__( self,
                  intakeSys:Intake,
                  setPos:Intake.IntakePositions|degrees
                ) -> None:
        # Command Attributes
        self.intake_sys:Intake = intakeSys
        self.setPos = setPos

        self.setName( f"ControlPivotPos: {setPos} degrees" )
        self.addRequirements( intakeSys )

    def initialize(self) -> None:
        self.intake_sys.setPivotSetpoint(self.setPos)

    def execute(self) -> None:
        pass

    def end(self, interrupted:bool) -> None:
        pass

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False