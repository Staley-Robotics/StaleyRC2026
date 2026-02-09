from commands2 import Subsystem
from wpilib import RobotState
from ntcore import NetworkTable, NetworkTableInstance
from phoenix6 import configs, controls, hardware, signals


class IntakeSpin(Subsystem):
    def __init__(self, motor_id:int):
        self.spin_motor = hardware.TalonFX(motor_id, "rio")
        self.intake_speed = 0
    
    def periodic(self) -> None:
        # Logging: Write Current Subsystem State
        # self.m_logging.putNumber( "SubsystemData", 0.0 )

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled():
            self.stop()
        else:
            self.run()
        
        # Logging: Write Post Operation Information
        # self.m_logging.putNumber( "Setpoint", self.getSetpoint() )
        # self.m_logging.putNumber( "Measured", self.m_system )

    # Run the Subsystem
    def run(self) -> None:
        self.spin_motor.set(self.intake_speed)

    # Stop the Subsystem
    def stop(self) -> None:
        self.spin_motor.set(0)

    # Set the Desired State Value
    def setSpeed(self, value:float) -> None:
        """
        Sets the desired intake speed to value
        """
        self.intake_speed = value

    # Get the Desired State Value
    def getSpeed(self) -> float:
        """
        Returns the current intake set speed
        """
        return self.intake_speed


# intake_exampl = Intake(1)