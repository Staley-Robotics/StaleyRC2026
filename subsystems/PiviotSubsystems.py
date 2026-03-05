from commands2 import Subsystem
from wpilib import RobotState, Color, Color8Bit, Mechanism2d, SmartDashboard
from ntcore import NetworkTable, NetworkTableInstance
from wpimath.system.plant import DCMotor
from wpilib.simulation import SingleJointedArmSim

from rev import SparkMax, SparkMaxSim, SparkMaxConfig, AlternateEncoderConfig, ClosedLoopConfig, ClosedLoopSlot, AbsoluteEncoderConfig, AbsoluteEncoder, SparkClosedLoopController

from wpimath.units import *

from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import PositionVoltage
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue

class PiviotConstants:
    MOTOR_ID = 3 #change to ur liking
    SPEED = 0.8
    GEAR_RATIOS = 5.45



class Piviot(Subsystem,):

    def __init__(self):
        #simulation
        self.motor = TalonFX(PiviotConstants.MOTOR_ID, "rio")
        self.motor.setNeutralMode(NeutralModeValue.BRAKE)

        self.simEncoder = self.simMotor.getAlternateEncoderSim()
        self.simEncoder.setPosition( degreesToRotations(-90) )

        self.simMotor = TalonFX( self.motor, DCMotor.krakenX60() )
        self.armSim = SingleJointedArmSim(
        DCMotor.krakenX60(), PiviotConstants.GEAR_RATIOS, 
        SingleJointedArmSim.estimateMOI( 0,0 ),
        degreesToRadians( PiviotPOS.MIN ),
        True, degreesToRadians(PiviotPOS.START),

        )

    def periodic(self):
        # safety + logging
        pass

class PiviotPOS():
    MIN:degrees = -74
    MAX:degrees = 90
    START:degrees = -90
    SOURCE:degrees = 5
    HOLD:degrees = 90