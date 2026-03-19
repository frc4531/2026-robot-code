import ntcore
import rev
import wpilib
from commands2 import SubsystemBase


class ClimberSubsystem(SubsystemBase):
    # Create a new LiftSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.climber_motor = rev.SparkFlex(6, rev.SparkMax.MotorType.kBrushless)

        self.climber_encoder = self.climber_motor.getEncoder()

        nt_instance = ntcore.NetworkTableInstance.getDefault()
        hopper_table = nt_instance.getTable("climber_table")

        self.position_entry = hopper_table.getDoubleTopic("climber_position").publish()

    def periodic(self):
        self.position_entry.set(self.get_climber_position())

    def set_climber_speed(self, speed):
        self.climber_motor.set(speed)

    def get_climber_position(self):
        return self.climber_encoder.getPosition()
