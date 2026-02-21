import commands2
import wpilib
import wpimath.controller

from subsystems.turret_subsystem import TurretSubsystem
from subsystems.vision_subsystem import VisionSubsystem


class TurretToPosition(commands2.PIDCommand):

    def __init__(self, turret_sub: TurretSubsystem, vision_sub: VisionSubsystem, target_position) -> None:
        super().__init__(
            wpimath.controller.PIDController(0.0065, 0.015, 0),
            # Close loop on absolute encoder
            lambda: vision_sub.turret_angle,
            # Set reference to target
            target_position,
            # Pipe output to turn arm
            lambda output: turret_sub.set_turret_speed(-output),
            # Require the arm
            turret_sub, vision_sub
        )

    def isFinished(self) -> bool:
        return False