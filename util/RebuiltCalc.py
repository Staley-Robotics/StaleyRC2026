import typing
from enum import Enum, auto

from wpilib import DriverStation, Field2d, SmartDashboard
from wpimath.geometry import Pose2d, Translation2d, Translation3d, Rotation2d, Twist2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import meters, inchesToMeters, seconds

from ntcore.util import ntproperty

from robotpy_apriltag import AprilTagFieldLayout

from phoenix6.swerve import SwerveDrivetrain
from phoenix6.utils import get_current_time_seconds
from phoenix6.units import *

from util import FalconLogger

'''
field dims (welded) (inches) (considering 0,0 bottom left) (assuming blue is right(?))
using: https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
width: 317.69
length: 651.22

left trench line (score zone): 182.11
right trench line (score zone): width-wall_to_trench=469.11

centerY (intersects hubs): 158.84

estimates:
relayLeft (from left): (145.6 (4/5*wall_to_trench), centerY+75.93=234.77)
relayRight (from left): (145.6, centerY-75.93=82.91)

relayLeft (from right): (505.62 (length - 4/5*wall_to_trench), 234.77)
relayRight (from right): (505.62, 82.91)
'''
# apl = AprilTagFieldLayout('./deploy/apriltags/2026-official.json')
class TargetPoints:
    """
    Points we intend to launch fuel at in Rebuilt
    See above in this file for more field dim info

    Stored as Pose2ds to allow pretty
    """
    # apl = AprilTagFieldLayout('./deploy/apriltags/2026-official.json')

    # apriltag version
    relayRightRed:Pose2d=Pose2d(inchesToMeters(505.62), inchesToMeters(234.77), 0)
    relayLeftRed:Pose2d=Pose2d(inchesToMeters(505.62), inchesToMeters(82.91), 0)
    relayLeftBlue:Pose2d=Pose2d(inchesToMeters(145.6), inchesToMeters(234.77), 0)
    relayRightBlue:Pose2d=Pose2d(inchesToMeters(145.6), inchesToMeters(82.91), 0)
    
    redHub:Pose2d=Pose2d(11.9154194, 4.0346376, 0)
    blueHub:Pose2d=Pose2d(4.6256194, 4.0346376, 0)

    # redHub:Pose2d=Pose2d(apl.getTagPose(2).x, apl.getTagPose(10).y, 0)
    # blueHub:Pose2d=Pose2d(apl.getTagPose(18).x, apl.getTagPose(26).y, 0)

    # estimated version:
    # relayRightRed:Translation2d=Translation2d(inchesToMeters(505.62), inchesToMeters(234.77))
    # relayLeftRed:Translation2d=Translation2d(inchesToMeters(505.62), inchesToMeters(82.91))
    # relayLeftBlue:Translation2d=Translation2d(inchesToMeters(145.6), inchesToMeters(234.77))
    # relayRightBlue:Translation2d=Translation2d(inchesToMeters(145.6), inchesToMeters(82.91))
    
    # redHub:Pose2d=Pose2d(inchesToMeters(469.11), inchesToMeters(148.84), 0)
    # blueHub:Pose2d=Pose2d(inchesToMeters(182.11), inchesToMeters(148.84), 0)

class RelayTarget(Enum):
    LEFT=auto()
    RIGHT=auto()
    AUTO=auto()

class FieldBoundaries:
    """
    Delimiters for certain zones on the field
    """
    redScoreZoneX:meters=12.51917739999 + 0.5
    blueScoreZoneX:meters=4.0218614 - 0.5
    
    centerLineY:meters=4.0346376

    # redScoreZoneX:meters=apl.getTagPose(10).x + 0.5
    # blueScoreZoneX:meters=apl.getTagPose(26).x - 0.5
    
    # centerLineY:meters=apl.getTagPose(26).y

class LaunchingConstants:
    launcherOffset:Translation2d = Translation2d() #TODO: measure
    '''The offset of the launcher from the center of the robot'''
    launchTime:seconds = 0.6 #ntproperty("Settings/RebuiltCalc/launchTime", 0.2, persistent=True) #TODO: measure
    '''The average time it takes for a fuel to move from the agitator to leaving the launcher'''

class RebuiltCalc:
    # variable definitions
    swerveSys:SwerveDrivetrain = None
    getRobotPose:typing.Callable[[], Pose2d] = lambda:Pose2d()
    getRobotState:typing.Callable[[], Pose2d] = lambda:Pose2d()

    desiredRelayPoint:TargetPoints|None = None

    field_display = Field2d()

    '''
    defining all variables here in the class definition rather than __init__ means their values will be updated and accessible through the class
    this means if you define a RebuiltCalc() object in one place, referencing the RebuiltCalc class should provide the same data
    '''

    @classmethod
    def assignSwerveSys(cls, swerveSys:SwerveDrivetrain):
        cls.swerveSys = swerveSys
        cls.getRobotPose:typing.Callable[[], Pose2d] = lambda: swerveSys.get_state().pose
        cls.getRobotState:typing.Callable[[], SwerveDrivetrain.SwerveDriveState] = swerveSys.get_state

        SmartDashboard.putData('RebuiltCalcField', cls.field_display)
    
    @classmethod
    def debugLog(cls) -> None:
        pose:Pose2d = cls.getRobotPose()
        FalconLogger.logOutput("/RebuiltCalc/gotPose", pose)
        FalconLogger.logOutput("/RebuiltCalc/inScoreZone", cls.botInScoreZone())
        FalconLogger.logOutput("/RebuiltCalc/isLeft", cls.botIsLeft())
        FalconLogger.logOutput("/RebuiltCalc/currentTargetPose", cls.getCurrentTargetPose())
        FalconLogger.logOutput("/RebuiltCalc/estimatedPoseAtLaunch", cls.getEstimatedPoseAtLaunchTime())
        FalconLogger.logOutput("/RebuiltCalc/currentTargetName", cls.getCurrentTargetName())
        FalconLogger.logOutput("/RebuiltCalc/2dDistToTarget", cls.getDistToTarget())
        FalconLogger.logOutput("/RebuiltCalc/targetRotation", cls.getRotToTarget().degrees())
        FalconLogger.logOutput("/RebuiltCalc/rotToTarget", cls.getRotToTarget().degrees() - pose.rotation().degrees() - 180)

        cls.field_display.setRobotPose(pose)
        cls.field_display.getObject('crntTarget').setPose(cls.getCurrentTargetPose())
        cls.field_display.getObject('poseAtLaunch').setPose(cls.getEstimatedPoseAtLaunchTime())
    
    @classmethod
    def setDesiredRelay(cls, relay:RelayTarget) -> None:
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            if relay == RelayTarget.AUTO:
                cls.desiredRelayPoint = None
            elif relay == RelayTarget.LEFT:
                cls.desiredRelayPoint = TargetPoints.relayLeftBlue
            else:
                cls.desiredRelayPoint = TargetPoints.relayRightBlue
        else:
            if relay == RelayTarget.AUTO:
                cls.desiredRelayPoint = None
            elif relay == RelayTarget.LEFT:
                cls.desiredRelayPoint = TargetPoints.relayLeftRed
            else:
                cls.desiredRelayPoint = TargetPoints.relayRightRed
    
    @classmethod
    def botInScoreZone(cls) -> bool:
        pose = cls.getRobotPose()
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            return pose.x < FieldBoundaries.blueScoreZoneX
        else:
            return pose.x > FieldBoundaries.redScoreZoneX
    
    @classmethod
    def botIsLeft(cls) -> bool:
        pose = cls.getRobotPose()
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            return pose.y > FieldBoundaries.centerLineY
        else:
            return pose.y < FieldBoundaries.centerLineY
    
    @classmethod
    def getCurrentTargetPose(cls) -> Pose2d:
        """
        gets the Translation2d (point on the field) of the current target
        """
        point = None
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            point = TargetPoints.blueHub
        else:
            point = TargetPoints.redHub
        # if cls.botInScoreZone():
        #     if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
        #         point = TargetPoints.blueHub
        #     else:
        #         point = TargetPoints.redHub
        # elif not (cls.desiredRelayPoint is None):
        #     point = cls.desiredRelayPoint
        # else:
        #     if cls.botIsLeft():
        #         if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
        #             point = TargetPoints.relayLeftBlue
        #         else:
        #             point = TargetPoints.relayLeftRed
        #     else:
        #         if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
        #             point = TargetPoints.relayRightBlue
        #         else:
        #             point = TargetPoints.relayRightRed

        return point
    
    @classmethod
    def getCurrentTargetName(cls) -> str:
        """
        gets the Translation2d (point on the field) of the current target
        """
        if cls.botInScoreZone():
            if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
                return "blueHub"
            else:
                return "redHub"
        
        if not (cls.desiredRelayPoint is None):
            return "desiredRelayPoint"
        else:
            if cls.botIsLeft():
                if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
                    return "relayLeftBlue"
                else:
                    return "relayLeftRed"
            else:
                if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
                    return "relayRightBlue"
                else:
                    return "relayRightRed"
    
    ### Fuel Launching Calculations
    @classmethod
    def getEstimatedPoseAtLaunchTime(cls) -> Pose2d:
        '''
        returns the estimated pose of the robot at the time a theoretical fuel would launch

        time the fuel would launch is current time + estimated time from agitation to ejection from launcher
        '''
        # // Calculate estimated pose while accounting for phase delay
        estimatedPose:Pose2d = cls.getRobotPose()
        robotSpeeds:ChassisSpeeds = cls.getRobotState().speeds
        return estimatedPose.exp(
            Twist2d(
                robotSpeeds.vx * LaunchingConstants.launchTime,
                robotSpeeds.vy * LaunchingConstants.launchTime,
                robotSpeeds.omega * LaunchingConstants.launchTime)
            )
                
    @classmethod
    def getDistToTarget(cls) -> meters:
        """
        returns the 2-dimensional distance from the robot to the currentTarget
        """
        # crnt_pose = Translation3d(cls.getRobotPose().x, cls.getRobotPose().y, 0.0) # assume robot is on ground
        return cls.getEstimatedPoseAtLaunchTime().translation().distance(cls.getCurrentTargetPose().translation())

    @classmethod
    def getRotToTarget(cls) -> Rotation2d:
        '''
        gets the Rotation2d to the current target relative to the field based on current robot translation
        '''
        target = cls.getCurrentTargetPose()
        rob_pose = cls.getEstimatedPoseAtLaunchTime()

        #this probably works idk
        dX = target.x - rob_pose.x
        dY = target.y - rob_pose.y
        goalAngle = Rotation2d( x = -dX, y = -dY ) # these negatives are wrong, but I'm dealing with them other places so they stay for now

        return goalAngle
    

