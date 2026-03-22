import typing

from commands2 import Command, Subsystem
from wpimath.geometry import Rotation2d

from phoenix6.units import *

from subsystems import SwerveDrive

class DriveByStick(Command):
    # Variable Declaration
    # swerve_sys:SwerveDrive = None
    # m_getValue:typing.Callable[[],float] = lambda: 0.0
    
    # Initialization
    def __init__( self,
                  swerveSys: SwerveDrive,
                  getX: typing.Callable[[], float] = lambda: 0.0,
                  getY: typing.Callable[[], float] = lambda: 0.0,
                  getRot: typing.Callable[[], float] = lambda: 0.0
                ) -> None:
        # Command Attributes
        self.swerve_sys:SwerveDrive = swerveSys
        self.get_x = getX
        self.get_y = getY
        self.get_rot = getRot

        self.field_centric = True

        self.desired_rot: typing.Callable[[], Rotation2d] = lambda: Rotation2d()
        self.use_desired_rot = False

        self.setName( f"{self.__class__.__name__}" )
        self.addRequirements( swerveSys )

    # On Start
    def initialize(self) -> None:
        pass

    # Periodic
    def execute(self) -> None:
        pass

    # On End
    def end(self, interrupted:bool) -> None:
        pass

    # Is Finished
    def isFinished(self) -> bool:
        return False

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False
    
    def setDriveWithRot(self, enabled:bool, switch_desired_rot:typing.Callable[[], Rotation2d]) -> None:
        self.use_desired_rot = enabled
        self.desired_rot = switch_desired_rot
