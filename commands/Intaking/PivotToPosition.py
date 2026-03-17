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

        self.setName( f"PivotToPosition: {setPos} degrees" )
        self.addRequirements( intakeSys )

    def initialize(self) -> None:
        pass
        # self.intake_sys.setPivotSetpoint(self.setPos)

    def execute(self) -> None:
        self.intake_sys.setPivotSetpoint(self.setPos)

    def end(self, interrupted:bool) -> None:
        pass
        # if interrupted:
        # self.intake_sys.setPivotSetpoint(self.intake_sys.getPivotPosition())

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False