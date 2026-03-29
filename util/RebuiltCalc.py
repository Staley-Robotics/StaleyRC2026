import typing
from enum import Enum, auto

from wpilib import DriverStation
from wpimath.geometry import Pose2d, Translation2d, Translation3d, Rotation2d
from wpimath.units import meters, inchesToMeters

from phoenix6.swerve import SwerveDrivetrain

from util import FalconLogger

'''
field dims (welded) (inches) (considering 0,0 bottom left) (assuming blue is right)
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

class TargetPoints:
    """
    Points we intend to launch fuel at in Rebuilt
    See above in this file for more field dim info
    """
    relayRightRed:Translation2d=Translation2d(inchesToMeters(505.62), inchesToMeters(234.77))#TODO: check if correct
    relayLeftRed:Translation2d=Translation2d(inchesToMeters(505.62), inchesToMeters(82.91))#TODO: check if correct
    relayLeftBlue:Translation2d=Translation2d(inchesToMeters(145.6), inchesToMeters(234.77))#TODO: check if correct
    relayRightBlue:Translation2d=Translation2d(inchesToMeters(145.6), inchesToMeters(82.91))#TODO: check if correct
    
    redHub:Translation2d=Translation2d(inchesToMeters(469.11), inchesToMeters(148.84))#TODO: check if correct
    blueHub:Translation2d=Translation2d(inchesToMeters(182.11), inchesToMeters(148.84))#TODO: check if correct

class RelayTarget(Enum):
    LEFT=auto()
    RIGHT=auto()
    AUTO=auto()

class FieldBoundaries:
    """
    Delimiters for certain zones on the field
    """
    redScoreZoneX:meters=inchesToMeters(469.11) #TODO: check if correct
    blueScoreZoneX:meters=inchesToMeters(182.11) #TODO: check if correct
    
    centerLineY:meters=inchesToMeters(158.84) #TODO: check if correct

class RebuiltCalc:
    _instance:typing.Self=None

    # variable definitions
    getRobotPose:typing.Callable[[], Pose2d] = lambda:Pose2d()
    getRobotState:typing.Callable[[], Pose2d] = lambda:Pose2d()

    desiredRelayPoint:TargetPoints|None = None

    '''
    defining all variables here in the class definition rather than __init__ means their values will be updated and accessible through the class
    this means if you define a RebuiltCalc() object in one place, referencing the RebuiltCalc class should provide the same data
    '''
    
    @classmethod
    def getInst(cls):
        if cls._instance is None:
            cls._instance = RebuiltCalc()
        return cls._instance
    
    @classmethod
    def setGetRobotPose(cls, getRobotState:typing.Callable[[], SwerveDrivetrain.SwerveDriveState]):
        cls.getRobotPose:typing.Callable[[], Pose2d] = lambda: getRobotState().pose
        cls.getRobotState:typing.Callable[[], SwerveDrivetrain.SwerveDriveState] = getRobotState
    
    @classmethod
    def debugLog(cls) -> None:
        pose:Pose2d = cls.getRobotPose()
        FalconLogger.logOutput("/RebuiltCalc/gotPose", pose)
        FalconLogger.logOutput("/RebuiltCalc/inScoreZone", cls.botInScoreZone())
        FalconLogger.logOutput("/RebuiltCalc/isLeft", cls.botIsLeft())
        FalconLogger.logOutput("/RebuiltCalc/currentTargetPoint", cls.getCurrentTargetPoint())
        FalconLogger.logOutput("/RebuiltCalc/currentTarget", cls.getCurrentTarget())
        FalconLogger.logOutput("/RebuiltCalc/2dDistToTarget", cls.getDistToTarget())
        FalconLogger.logOutput("/RebuiltCalc/targetRotation", cls.getRotToTarget().degrees())
        FalconLogger.logOutput("/RebuiltCalc/rotToTarget", cls.getRotToTarget().degrees() - pose.rotation().degrees() - 180)
    
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
    def getCurrentTargetPoint(cls) -> Translation2d:
        """
        gets the Translation2d (point on the field) of the current target
        """
        point = None
        if cls.botInScoreZone():
            if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
                point = TargetPoints.blueHub
            else:
                point = TargetPoints.redHub
        elif not (cls.desiredRelayPoint is None):
            point = cls.desiredRelayPoint
        else:
            if cls.botIsLeft():
                if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
                    point = TargetPoints.relayLeftBlue
                else:
                    point = TargetPoints.relayLeftRed
            else:
                if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
                    point = TargetPoints.relayRightBlue
                else:
                    point = TargetPoints.relayRightRed

        # state:SwerveDrivetrain.SwerveDriveState = cls.getRobotState()
        # point = Translation2d(point.x + state.speeds.vx * 2, point.y + state.speeds.vy * 2)

        return point
    
    @classmethod
    def getCurrentTarget(cls) -> str:
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
                
    @classmethod
    def getDistToTarget(cls) -> meters:
        """
        returns the 2-dimensional distance from the robot to the currentTarget
        """
        # crnt_pose = Translation3d(cls.getRobotPose().x, cls.getRobotPose().y, 0.0) # assume robot is on ground
        return cls.getRobotPose().translation().distance(cls.getCurrentTargetPoint())

    @classmethod
    def getRotToTarget(cls) -> Rotation2d:
        '''
        gets the Rotation2d to the current target relative to the field based on current robot translation
        '''
        target = cls.getCurrentTargetPoint()

        #this probably works idk
        dX = target.x - cls.getRobotPose().x
        dY = target.y - cls.getRobotPose().y
        goalAngle = Rotation2d( x = -dX, y = -dY )

        return goalAngle
    
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


'''
Target system:
- used to get:
    - bot rotation towards target
    - distance (3d?) to target
- targets could be:
    - score
    - relayLeft
    - relayRight (direction is from driver perspective)

if not bot_in_score_zone:
    targetType = relay
    if desiredRelay != None:
        return desiredRelay
    else:
        if bot_in_left_zone: relayLeft
        else (bot_in_right_zone): relayRight
'''
    

