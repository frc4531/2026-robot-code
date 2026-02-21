import commands2
import rev
from rev import SparkFlexConfig, SparkBase

from subsystems.shooter_subsystem import ShooterSubsystem


class ShooterToVelocity(commands2.Command):

    def __init__(self, shooter_sub: ShooterSubsystem, target_position) -> None:
        super().__init__()

        self.target_position = target_position

        self.shooter_sub = shooter_sub
        self.addRequirements(self.shooter_sub)

    def execute(self) -> None:
        self.shooter_sub.set_shooter_velocity(self.target_position)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.shooter_sub.set_shooter_speed(0)