import commands2
import wpilib
import wpimath.controller

from subsystems.hopper_subsystem import HopperSubsystem


class ExtensionToPosition(commands2.PIDCommand):

    def __init__(self, hopper_sub: HopperSubsystem, target_position) -> None:
        super().__init__(
            wpimath.controller.PIDController(1.25, 0, 0),
            # Close loop on absolute encoder
            lambda: hopper_sub.get_extension_position(),
            # Set reference to target
            target_position,
            # Pipe output to turn arm
            lambda output: hopper_sub.set_extension_speed(output),
            # Require the arm
            hopper_sub
        )

    def isFinished(self) -> bool:
        return False