import typing

from commands2 import Command, Subsystem
# from wpimath.units import percent
from ntcore.util import ntproperty

from subsystems import Intake

class RunIntakeByNT(Command):
    # Variable Declaration
    intake_sys:Intake = None

    speed = ntproperty("/Settings/RunIntakeByNT/speed (rps: -1 to 1)", Intake.Speeds.IN, persistent=True)
    
    # Initialization
    def __init__( self,
                  intakeSys:Intake,
                ) -> None:
        # Command Attributes
        self.intake_sys:Intake = intakeSys

        self.setName( f"RunIntakeByNT" )
        self.addRequirements( intakeSys )

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        self.intake_sys.setIntakeSpeed( self.speed )

    def end(self, interrupted:bool) -> None:
        self.intake_sys.setIntakeSpeed(Intake.Speeds.STOP)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False