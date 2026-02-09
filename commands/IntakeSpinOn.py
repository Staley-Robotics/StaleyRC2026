import typing

from commands2 import Command, Subsystem
from subsystems.IntakeSpin import IntakeSpin 

class IntakeSpinOn(Command):
    # Variable Declaration
    m_Spin:IntakeSpin = None
    #m_getValue:typing.Callable[[],float] = lambda: 0.0
    
    # Initialization
    def __init__( self,
                  mySubsystem:IntakeSpin,
                  #myValue: typing.Callable[[], float] = lambda: 0.0
                ) -> None:
        # Command Attributes
        self.m_Spin:IntakeSpin = mySubsystem
        #self.m_getValue = myValue
        self.setName( "IntakeSpinOn" )
        self.addRequirements( mySubsystem )

    # On Start
    def initialize(self) -> None:
        pass

    # Periodic
    def execute(self) -> None:
        #self.m_subsystem.setSetpoint( self.m_getValue() )
        self.m_Spin.setSpeed(-0.5)

    # On End
    def end(self, interrupted:bool) -> None:
        self.m_Spin.setSpeed(0)

    # Is Finished
    def isFinished(self) -> bool:
        return False

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False