import typing

from commands2 import Command, Subsystem
from subsystems import Climber, ClimberPositions
from ntcore.util import ntproperty

class ControlClimberPosition(Command):
    # Variable Declaration
    climber_Sys:Climber = None
    # m_getValue:typing.Callable[[],float] = lambda: 0.0
    controlSpeed = ntproperty('/Settings/Elevator/ControlSpeed', 1, persistent=True)
    
    # Initialization
    def __init__( self,
                  ClimberSys:Subsystem,
                #   joystick: typing.Callable[[], float] = lambda: 0.0
                ) -> None:
        # Command Attributes
        self.climber_Sys:Climber = ClimberSys
        # self.m_getValue = joystick
        self.setName( "ControlClimberPosition" )
        self.addRequirements( ClimberSys )

    # On Start
    def initialize(self) -> None:
        pass

    # Periodic
    def execute(self) -> None:
        self.setPos = max(min(self.climber_Sys.getSetpoint() + self.getPos() * self.controlSpeed, ClimberPositions.TOP), ClimberPositions.BOTTOM)
        self.climber_Sys.changeDesiredPos(self.setPos)
    # On End
    def end(self, interrupted:bool) -> None:
        pass

    # Is Finished
    def isFinished(self) -> bool:
        return False

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False