import typing
from enum import Enum

from wpilib import DriverStation
from wpimath.geometry import Pose2d, Translation2d, Translation3d, Rotation2d
from wpimath.units import meters, inchesToMeters

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

class FieldBoundaries:
    """
    Delimiters for certain zones on the field
    """
    redScoreZoneX:meters=inchesToMeters(469.11) #TODO: check if correct
    blueScoreZoneX:meters=inchesToMeters(182.11) #TODO: check if correct
    
    centerLineY:meters=inchesToMeters(158.84) #TODO: check if correct

class RebuiltCalc:
    getRobotPose:typing.Callable[[], Pose2d] = lambda:Pose2d()

    desiredRelayPoint:TargetPoints|None = None

    def __init__(self, getRobotPose:typing.Callable[[], Pose2d]):
        self.getRobotPose = getRobotPose
    
    def debugLog(self) -> None:
        FalconLogger.logOutput("/RebuiltCalc/gotPose", self.getRobotPose())
        FalconLogger.logOutput("/RebuiltCalc/inScoreZone", self.botInScoreZone())
        FalconLogger.logOutput("/RebuiltCalc/isLeft", self.botIsLeft())
        FalconLogger.logOutput("/RebuiltCalc/currentTarget", self.getCurrentTargetPoint())
        FalconLogger.logOutput("/RebuiltCalc/2dDistToTarget", self.getDistToTarget())
        FalconLogger.logOutput("/RebuiltCalc/rotToTarget", self.getRotToTarget().degrees())
    
    def botInScoreZone(self) -> bool:
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            return self.getRobotPose().x < FieldBoundaries.blueScoreZoneX
        else:
            return self.getRobotPose().x > FieldBoundaries.redScoreZoneX
    
    def botIsLeft(self) -> bool:
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            return self.getRobotPose().y > FieldBoundaries.centerLineY
        else:
            return self.getRobotPose().y < FieldBoundaries.centerLineY
        
    def getCurrentTargetPoint(self) -> Translation2d:
        """
        gets the Translation2d (point on the field) of the current target
        """
        if self.botInScoreZone():
            if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
                return TargetPoints.blueHub
            else:
                return TargetPoints.redHub
        
        if not (self.desiredRelayPoint is None):
            return self.desiredRelayPoint
        else:
            if self.botIsLeft():
                if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
                    return TargetPoints.relayLeftBlue
                else:
                    return TargetPoints.relayLeftRed
            else:
                if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
                    return TargetPoints.relayRightBlue
                else:
                    return TargetPoints.relayRightRed
                
    
    def getDistToTarget(self) -> meters:
        """
        returns the 2-dimensional distance from the robot to the currentTarget
        """
        # crnt_pose = Translation3d(self.getRobotPose().x, self.getRobotPose().y, 0.0) # assume robot is on ground
        return self.getRobotPose().translation().distance(self.getCurrentTargetPoint())

    def getRotToTarget(self) -> Rotation2d:
        target = self.getCurrentTargetPoint()

        #this probably works idk
        dX = target.x - self.getRobotPose().x
        dY = target.y - self.getRobotPose().y
        goalAngle = Rotation2d( x = -dX, y = -dY )

        return goalAngle


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
    

