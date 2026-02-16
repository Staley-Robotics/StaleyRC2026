from commands2 import Subsystem
from wpilib import RobotState, Color, Color8Bit, Mechanism2d, SmartDashboard
from ntcore import NetworkTable, NetworkTableInstance

from wpilib.simulation import SingleJointedArmSim
from wpimath.system.plant import DCMotor

from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import PositionVoltage
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue


class IntakePivotConstants:
    # Pivot geometry / conversion
    GEAR_RATIO = 100.0  # motor rotations per 1 pivot rotation
    LOW_ANGLE_DEG = 20.0
    HIGH_ANGLE_DEG = 75.0
    MIN_ANGLE_DEG = 0.0
    MAX_ANGLE_DEG = 90.0

    # Approximate mechanism properties (realistic defaults)
    ARM_LENGTH_M = 0.65
    ARM_MASS_KG = 4.0

    # Simple simulation voltage control toward target
    SIM_KP_VOLTS_PER_DEG = 0.20

class IntakePivot(Subsystem):
    # Variable Declaration
    m_motor:int = None
    m_logging:NetworkTable = None

    # Initialization
    def __init__(self, motor_Id:int) -> None:
        self.m_motor = TalonFX(motor_Id, "rio")
        self.m_logging = NetworkTableInstance.getDefault().getTable("/Logging/IntakePivot")

    # Periodic Loop
    def periodic(self) -> None:
        # Logging: Write Current Subsystem State
        self.m_logging.putNumber( "SubsystemData", 0.0 )

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled():
            self.stop()
        else:
            self.run()
        
        # Logging: Write Post Operation Information
        self.m_logging.putNumber( "Setpoint", self.getSetpoint() )
        self.m_logging.putNumber( "Measured", self.m_system )

    # Run the Subsystem
    def run(self) -> None:
        pass

    # Stop the Subsystem
    def stop(self) -> None:
        pass

    # Set the Desired State Value
    def setSetpoint(self, value:float) -> None:
        self.m_value = value

    # Get the Desired State Value
    def getSetpoint(self) -> float:
        return self.m_value
    
    # Check if Subsystem is at the Desired State
    def atSetpoint(self) -> bool:
        return False