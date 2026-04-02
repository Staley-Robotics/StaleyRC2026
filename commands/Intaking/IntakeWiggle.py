import typing

from commands2 import Command, Subsystem
from wpimath.units import degrees
from ntcore.util import ntproperty
from subsystems import Intake, IntakeConstants
from util.FalconLogger import FalconLogger


class IntakeWiggle(Command):
    # Variable Declaration
    intake_sys:Intake = None
    
    # Initialization
    def __init__( self,
                  intakeSys:Intake,
                  bottomPos:Intake.Positions|degrees,
                  topPos:Intake.Positions|degrees
                ) -> None:
        # Command Attributes
        self.intake_sys:Intake = intakeSys
        self.bottomPos = bottomPos
        self.topPos = topPos
        self.setName( f"IntakeWiggle" )
        self.addRequirements( intakeSys )

    def initialize(self) -> None:
        self.intake_sys.setPivotSetpoint(self.topPos)
        self.lastPos = self.intake_sys.getPivotSetpoint()

        self.intake_sys.setIntakeSpeed(Intake.Speeds.IN)
        
        # self.intake_sys.setPivotSetpoint(self.setPos)

    def execute(self) -> None:
        if self.isAtSetpoint():
            if self.lastPos == self.topPos:
                self.intake_sys.setPivotSetpoint(self.bottomPos)
                self.lastPos = self.bottomPos
            elif self.lastPos == self.bottomPos:
                self.intake_sys.setPivotSetpoint(self.topPos)
                self.lastPos = self.topPos
        FalconLogger.logOutput("/Intake/Outputs/lastPos", self.lastPos)
        FalconLogger.logOutput("/Intake/Outputs/Wiggle At Setpoint", self.isAtSetpoint())
        
        """
        Pseudocode for Luke:
        1. At start, setSetpoint to topPos
        2. If we are at the setPoint, set the setPoint to the other setPoint (if top, then go bottom; vice versa)
        """

    def end(self, interrupted:bool) -> None:
        # pass
        # if interrupted:
        self.intake_sys.setPivotSetpoint(self.intake_sys.getPivotPosition())
        self.intake_sys.setIntakeSpeed(Intake.Speeds.STOP)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False
    
    def isAtSetpoint(self) -> bool:
        """
        Checks whether the intake is in the correct position W/O using the built-in Intake.getAtSetpoint() method
        To ben: using getAtSetpoint() isn't always accurate, so we used our own
        """
        return abs(self.intake_sys.getPivotPosition() - self.intake_sys.getPivotSetpoint()) < IntakeConstants.wiggle_tolerance