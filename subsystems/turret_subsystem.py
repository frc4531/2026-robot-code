import ntcore
import rev
import wpilib
from commands2 import SubsystemBase


class TurretSubsystem(SubsystemBase):
    # Create a new LiftSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.turret_motor = rev.SparkFlex(7, rev.SparkMax.MotorType.kBrushless)
        self.hood_motor = rev.SparkFlex(8, rev.SparkMax.MotorType.kBrushless)

        self.turret_motor.setInverted(True)
        self.hood_motor.setInverted(True)

    def set_turret_speed(self, speed):
        self.turret_motor.set(speed)

    def set_hood_speed(self, speed):
        self.hood_motor.set(speed)
