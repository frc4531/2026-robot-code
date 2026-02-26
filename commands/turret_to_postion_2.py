import commands2
import rev
from rev import SparkFlexConfig, SparkBase

from subsystems.turret_subsystem import TurretSubsystem
from wpimath.controller import PIDController

import ntcore

class TurretToPosition2(commands2.Command):

    def __init__(self, turret_sub: TurretSubsystem, target_position) -> None:
        super().__init__()

        self.target_position = target_position

        self.turret_sub = turret_sub
        self.addRequirements(self.turret_sub)

        self.turret_pid_controller = PIDController(0.016, 0, 0)

        self.turret_pid_controller.enableContinuousInput(-180, 180)

        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.vision_table = self.inst.getTable("vision_table")
        self.turret_position = self.vision_table.getFloatTopic("turret_position").subscribe(0.0)

    def execute(self) -> None:
        turret_angle = self.turret_position.get()

        output = self.turret_pid_controller.calculate(turret_angle, self.target_position)
        self.turret_sub.set_turret_speed(output)
        self.turret_sub.turret_pid_entry.set(output)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.turret_sub.set_turret_speed(0)