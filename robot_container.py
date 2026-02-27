import math

import ntcore
import wpilib
import commands2
from commands2 import WaitCommand

from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

from commands.drive_command import DriveCommand
from commands.extension_to_position import ExtensionToPosition
from commands.hood_down import HoodDown
from commands.hood_to_positon import HoodToPosition
from commands.hood_up import HoodUp
from commands.hopper_out import HopperOut
from commands.input_drive import InputDrive
from commands.intake_in import IntakeIn
from commands.shooter_out import ShooterOut
from commands.shooter_to_velocity import ShooterToVelocity
from commands.turret_left import TurretLeft
from commands.turret_right import TurretRight
from commands.turret_to_position import TurretToPosition
from constants.position_constants import PositionConstants
from constants.swerve_constants import OIConstants, AutoConstants, DriveConstants
from subsystems.drive_subsystem import DriveSubsystem
from subsystems.hopper_subsystem import HopperSubsystem
from subsystems.shooter_subsystem import ShooterSubsystem
from subsystems.turret_subsystem import TurretSubsystem
from subsystems.vision_subsystem import VisionSubsystem
from subsystems.intake_subsystem import IntakeSubsystem


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems
        self.drive_subsystem = DriveSubsystem()
        self.vision_subsystem = VisionSubsystem()
        self.intake_subsystem = IntakeSubsystem()
        self.shooter_subsystem = ShooterSubsystem()
        self.hopper_subsystem = HopperSubsystem()
        self.turret_subsystem = TurretSubsystem()

        # The driver's controller
        self.driver_controller = wpilib.Joystick(OIConstants.kDriverControllerPort)
        self.operator_controller = wpilib.Joystick(OIConstants.kOperatorControllerPort)

        # Configure the button bindings
        self.configure_button_bindings()

        # Configure default commands
        self.drive_subsystem.setDefaultCommand(
            DriveCommand(self.drive_subsystem)
        )
        # Configure Auto Chooser
        self.chooser = wpilib.SendableChooser()
        self.do_nothing = "Do Nothing"

        self.chooser.setDefaultOption("Shoot 1 Only", self.do_nothing)

        self.drive_subsystem.gyro.reset()

    def configure_button_bindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        # Intake In
        commands2.button.JoystickButton(self.operator_controller, 1).whileTrue(
            IntakeIn(self.intake_subsystem)
        )
        # Shooter Out
        commands2.button.JoystickButton(self.operator_controller, 3).toggleOnTrue(
            ShooterToVelocity(self.shooter_subsystem, 6500)
        )
        # Shooter Out
        commands2.button.JoystickButton(self.operator_controller, 4).whileTrue(
            ShooterToVelocity(self.shooter_subsystem, 2000)
        )
        # Hopper Out
        commands2.button.JoystickButton(self.operator_controller, 5).whileTrue(
            HopperOut(self.hopper_subsystem)
        )
        # Turret Left
        commands2.button.JoystickButton(self.operator_controller, 7).whileTrue(
            TurretLeft(self.turret_subsystem)
        )
        # # Hood Up
        # commands2.button.JoystickButton(self.operator_controller, 12).whileTrue(
        #     HoodToPosition(self.turret_subsystem, 5)
        # )
        # # Hood Down
        # commands2.button.JoystickButton(self.operator_controller, 13).whileTrue(
        #     HoodToPosition(self.turret_subsystem, 0.5)
        # )

        # Turret to one position
        commands2.button.JoystickButton(self.operator_controller, 9).whileTrue(
            TurretToPosition(self.turret_subsystem, self.vision_subsystem, -90)
        )
        # Turret to another position
        commands2.button.JoystickButton(self.operator_controller, 10).whileTrue(
            TurretToPosition(self.turret_subsystem, self.vision_subsystem, -180)
        )
        # Turret to another position
        commands2.button.JoystickButton(self.operator_controller, 11).whileTrue(
            TurretToPosition(self.turret_subsystem, self.vision_subsystem, -225)
        )

        # Manual Turret
        commands2.button.JoystickButton(self.operator_controller, 12).whileTrue(
            TurretLeft(self.turret_subsystem)
        )
        # Manual Turret
        commands2.button.JoystickButton(self.operator_controller, 13).whileTrue(
            TurretRight(self.turret_subsystem)
        )

        # # Extension to one position
        # commands2.button.JoystickButton(self.operator_controller, 9).whileTrue(
        #     ExtensionToPosition(self.hopper_subsystem, 0.47)
        # )
        #
        # # Extension to another position
        # commands2.button.JoystickButton(self.operator_controller, 10).whileTrue(
        #     ExtensionToPosition(self.hopper_subsystem, 0.63)
        # )

    def disable_pid_subsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

    def get_autonomous_command(self) -> commands2.command:
        # Create config for trajectory
        config = TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared,
        )
        # Add kinematics to ensure max speed is actually obeyed
        config.setKinematics(DriveConstants.kDriveKinematics)

        # Forward Auto Trajectory to follow. All units in meters.
        forward_trajectory = TrajectoryGenerator.generateTrajectory(
            # Start at the origin facing the +X direction
            Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            # Pass through these two interior waypoints, making an 's' curve path
            [],
            # End 3 meters straight ahead of where we started, facing forward
            Pose2d(0, -3, Rotation2d.fromDegrees(0)),
            config,
        )

        thetaController = ProfiledPIDControllerRadians(
            AutoConstants.kPThetaController,
            0,
            0,
            AutoConstants.kThetaControllerConstraints,
        )
        thetaController.enableContinuousInput(-math.pi, math.pi)

        holonomic_controller = HolonomicDriveController(PIDController(AutoConstants.kPXController, 0, 0),
                                                        PIDController(AutoConstants.kPYController, 0, 0),
                                                        thetaController)

        # forward_trajectory_command = commands2.SwerveControllerCommand(
        #     forward_trajectory,
        #     self.drive_subsystem.get_pose,  # Functional interface to feed supplier
        #     DriveConstants.kDriveKinematics,
        #     # Position controllers
        #     holonomic_controller,
        #     self.drive_subsystem.set_module_states(),
        #     (self.drive_subsystem,),
        # )

        # Start Auto Logic
        return commands2.ParallelDeadlineGroup(
            WaitCommand(1),
            InputDrive(self.drive_subsystem, 0.4, 0, 0)
        )