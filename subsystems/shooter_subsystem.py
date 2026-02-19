import ntcore
import rev
import wpilib
from commands2 import SubsystemBase
from rev import ResetMode, PersistMode, SparkFlexConfig, SparkFlex


class ShooterSubsystem(SubsystemBase):
    # Create a new LiftSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.left_shooter_motor = SparkFlex(9, rev.SparkMax.MotorType.kBrushless)
        self.right_shooter_motor = SparkFlex(10, rev.SparkMax.MotorType.kBrushless)

        self.right_shooter_config = SparkFlexConfig()

        self.right_shooter_config.follow(9, True)

        self.right_shooter_motor.configure(self.right_shooter_config,
                                        ResetMode.kResetSafeParameters,
                                        PersistMode.kPersistParameters)

    def set_shooter_speed(self, speed):
        self.left_shooter_motor.set(speed)
