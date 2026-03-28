import typing

from commands2 import Command, Subsystem
from wpimath.geometry import Rotation2d, Pose2d
from wpilib import getTime

from wpimath.units import *
from phoenix6.units import *
from phoenix6 import swerve
from phoenix6.utils import get_current_time_seconds
from phoenix6.swerve.utility.phoenix_pid_controller import PhoenixPIDController

from ntcore.util import ntproperty

from subsystems import SwerveDrive
from util import RebuiltCalc

class DriveFacingDirectionCustom(Command):
    # Variable Declaration
    drive_rot_kP: percent =  ntproperty("Settings/drive/customRotpid/kP", 2.00, persistent=True)
    drive_rot_kI: percent =  ntproperty("Settings/drive/customRotpid/kI", 0.00, persistent=True)
    drive_rot_kD: percent =  ntproperty("Settings/drive/customRotpid/kD", 0.00, persistent=True)
    
    # Initialization
    def __init__( self,
                  swerveSys: SwerveDrive,
                  getX: typing.Callable[[], float] = lambda: 0.0,
                  getY: typing.Callable[[], float] = lambda: 0.0,
                  getDesiredRot: typing.Callable[[], Rotation2d] = lambda: 0.0
                ) -> None:
        # Command Attributes
        self.swerve_sys:SwerveDrive = swerveSys
        self.get_x = getX
        self.get_y = getY
        self.get_desired_rot:typing.Callable[[], Rotation2d] = getDesiredRot

        # create drive request with constant customizations
        self.drive_req = (
            swerve.requests.FieldCentric()
                .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
        )

        # Rotation control setup
        self.heading_controller: PhoenixPIDController = PhoenixPIDController(self.drive_rot_kP, self.drive_rot_kI, self.drive_rot_kP)

        self.setName( f"{self.__class__.__name__}" )
        self.addRequirements( swerveSys )

    # On Start
    def initialize(self) -> None:
        pass

    # Periodic
    def execute(self) -> None:
        
        self.heading_controller.setPID(self.drive_rot_kP, self.drive_rot_kI, self.drive_rot_kP)

        self.swerve_sys.set_control(
            self.drive_req.with_velocity_x( -self.get_y() * self.swerve_sys.max_drive_speed )
                          .with_velocity_y( -self.get_x() * self.swerve_sys.max_drive_speed )
                          .with_rotational_rate(self.getRotationalRate())

                          .with_deadband( self.swerve_sys.translation_deadband )
                          .with_rotational_deadband( self.swerve_sys.rotation_deadband )

                        #   .with_target_direction( self.get_desired_rot().rotateBy(Rotation2d(math.pi)) )
                        #   .with_heading_pid( self.drive_rot_kP, self.drive_rot_kI, self.drive_rot_kD )
        )

    def getRotationalRate(self):
        angle_to_face = self.get_desired_rot()
        # if self.forward_perspective is ForwardPerspectiveValue.OPERATOR_PERSPECTIVE:
        # If we're operator perspective, rotate the direction we want to face by the angle
        # this needs to happen
        # angle_to_face = angle_to_face.rotateBy(parameters.operator_forward_direction)

        to_apply_omega = self.heading_controller.calculate(
            RebuiltCalc.getRobotPose().rotation().radians(),
            angle_to_face.radians(),
            get_current_time_seconds()
        )
        if self.swerve_sys.max_rot_speed > 0.0:
            if to_apply_omega > self.swerve_sys.max_rot_speed:
                to_apply_omega = self.swerve_sys.max_rot_speed
            elif to_apply_omega < -self.swerve_sys.max_rot_speed:
                to_apply_omega = -self.swerve_sys.max_rot_speed

        return to_apply_omega
    
    def withRot(self, getDesiredRot:typing.Callable[[], Rotation2d]) -> typing.Self:
        '''
        modifies getDesiredRot internally and returns self for conciseness
        '''
        self.get_desired_rot = getDesiredRot
        return self

    # On End
    def end(self, interrupted:bool) -> None:
        pass

    # Is Finished
    def isFinished(self) -> bool:
        return False

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False
