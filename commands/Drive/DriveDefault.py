import typing
from enum import Enum, auto

from commands2 import Command, SelectCommand
from wpimath.geometry import Rotation2d

from phoenix6.units import *

from commands import DriveByStick, DriveFacingDirection
from subsystems import SwerveDrive

class DriveType(Enum):
    Stick=auto()
    FacingDir=auto()

class DriveDefaultSelect(SelectCommand):
    '''
    
    '''
    # Variable Declaration
    currentDriveType:DriveType=DriveType.Stick
    
    # Initialization
    def __init__( self,
                  swerveSys: SwerveDrive,
                  getX: typing.Callable[[], float] = lambda: 0.0,
                  getY: typing.Callable[[], float] = lambda: 0.0,
                  getRot: typing.Callable[[], float] = lambda: 0.0,
                  getDesiredRot: typing.Callable[[], Rotation2d] = lambda: Rotation2d()
                #   DriveByStick:DriveByStick,
                #   DriveFacingDirection:DriveFacingDirection,
                ) -> None:
        self.driveByStick = DriveByStick(swerveSys, getX, getY, getRot)
        self.driveFacingDir = DriveFacingDirection(swerveSys, getX, getY, getDesiredRot)

        # Select Setup
        super().__init__(
            {
                DriveType.Stick:self.driveByStick,
                DriveType.FacingDir:self.driveFacingDir,
            },
            lambda: (self.currentDriveType)
        )

        # Command Attributes
        self.swerve_sys:SwerveDrive = swerveSys

        self.field_centric = True

        self.setName( f"{self.__class__.__name__}" )
        self.addRequirements( swerveSys )
    
    def setDriveType(self, driveType:DriveType, setGetDesiredRot:typing.Callable[[], Rotation2d]|None=None) -> None:
        self.currentDriveType = driveType
        if not setGetDesiredRot is None:
            self.setGetDesiredRot(setGetDesiredRot)
        
        self.setName(f'Default - {driveType.name}')

    def getDriveType(self, driveType:DriveType) -> None:
        self.currentDriveType = driveType
    
    def setGetDesiredRot(self, newGetDesiredRot:typing.Callable[[], Rotation2d]) -> None:
        self.driveFacingDir.get_desired_rot = newGetDesiredRot
    
    def toggleFieldRelative(self) -> None:
        self.driveByStick.toggleFieldCentric()
    
    def initialize(self):
        self.setName(f'Default - {self.currentDriveType.name}')
        return super().initialize()

    # Is Finished
    def isFinished(self) -> bool:
        return False

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False

__all__ = [
    "DriveDefaultSelect",
    "DriveType"
]