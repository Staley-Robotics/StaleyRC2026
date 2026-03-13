import typing

from commands2 import Command, Subsystem
from ntcore.util import ntproperty

from subsystems import ClimberOpenLoop

class ControlClimberOpenLoop(Command):
    # Variable Declaration
    climber_sys:ClimberOpenLoop = None

    # a value of 1 would change desired height by 50 inches per second, thats too fast
    # 1/50 is 1 inch per second, thats alright
    input_mult = ntproperty("/Settings/ControlClimberOpenLoop/input mult", 0.3, persistent=True)
    
    # Initialization
    def __init__( self,
                  climberSys:ClimberOpenLoop,
                  stickInput:typing.Callable[[], float]=lambda:0.0
                ) -> None:
        '''
        :param stickInput: should be in range [-1,1]
        '''
        # Command Attributes
        self.climber_sys:ClimberOpenLoop = climberSys
        self.stick_input = stickInput

        self.setName( f"ControlClimberOpenLoop" )
        self.addRequirements( climberSys )

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        self.climber_sys.setSpeed(
            self.stick_input() * self.input_mult
        )

    def end(self, interrupted:bool) -> None:
        pass

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False