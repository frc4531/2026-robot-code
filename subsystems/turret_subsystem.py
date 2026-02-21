import ntcore
import rev
import wpilib
from commands2 import SubsystemBase
from rev import PersistMode, ResetMode


class TurretSubsystem(SubsystemBase):
    # Create a new LiftSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.turret_motor = rev.SparkFlex(7, rev.SparkMax.MotorType.kBrushless)
        self.hood_motor = rev.SparkFlex(8, rev.SparkMax.MotorType.kBrushless)

        self.turret_motor.setInverted(True)
        self.hood_motor.setInverted(True)

        self.hood_encoder = self.hood_motor.getEncoder()
        self.turret_encoder = self.turret_motor.getEncoder()

        nt_instance = ntcore.NetworkTableInstance.getDefault()
        hopper_table = nt_instance.getTable("turret_table")

        self.hood_position_entry = hopper_table.getDoubleTopic("hood_position").publish()
        self.turret_position_entry = hopper_table.getDoubleTopic("turret_position").publish()

        self.hood_config = rev.SparkMaxConfig()
        self.turret_config = rev.SparkMaxConfig()

        self.hood_pid_controller = self.hood_motor.getClosedLoopController()
        self.turret_pid_controller = self.hood_motor.getClosedLoopController()

        self.khP = 1
        self.khI = 0
        self.khD = 0
        self.min_speed = -0.5
        self.max_speed = 0.5

        self.hood_config.closedLoop.P(self.khP).I(self.khI).D(self.khD)
        self.hood_config.closedLoop.outputRange(self.min_speed, self.max_speed)

        self.hood_motor.configure(self.hood_config,
                                       ResetMode.kResetSafeParameters,
                                       PersistMode.kPersistParameters)
        self.turret_motor.configure(self.turret_config,
                                        ResetMode.kResetSafeParameters,
                                        PersistMode.kPersistParameters)

    def periodic(self):
        self.hood_position_entry.set(self.get_hood_position())
        self.turret_position_entry.set(self.get_turret_position())

    def get_hood_position(self):
        return self.hood_encoder.getPosition()

    def get_turret_position(self):
        return self.turret_encoder.getPosition()

    def set_turret_speed(self, speed):
        self.turret_motor.set(speed)

    def set_hood_speed(self, speed):
        self.hood_motor.set(speed)
