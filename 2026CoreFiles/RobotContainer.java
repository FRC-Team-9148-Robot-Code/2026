package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

// The robot's name: Megatron
public class RobotContainer {

private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
private double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond);

private final SwerveRequest.FieldCentric drive =
new SwerveRequest.FieldCentric()
.withDeadband(MaxSpeed * 0.1)
.withRotationalDeadband(MaxAngularRate * 0.1)
.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

private final SwerveRequest.SwerveDriveBrake brake =
new SwerveRequest.SwerveDriveBrake();

private final SwerveRequest.PointWheelsAt point =
new SwerveRequest.PointWheelsAt();

/* Controllers */
private final Joystick m_driverJoystick = new Joystick(0);
private final CommandXboxController operatorController = new CommandXboxController(1);

/* Subsystems */
public final CommandSwerveDrivetrain drivetrain =
TunerConstants.createDrivetrain();

public final Shooter shooter = new Shooter();

public RobotContainer() {

/* REGISTER PATHPLANNER COMMANDS */
NamedCommands.registerCommand(
"shoot",shooter.shootCommand(() -> 1.0, false).withTimeout(1));

configureBindings();
}

private void configureBindings() {

drivetrain.setDefaultCommand(
drivetrain.applyRequest(() ->
drive.withVelocityX(-m_driverJoystick.getY() * MaxSpeed)
.withVelocityY(-m_driverJoystick.getX() * MaxSpeed)
.withRotationalRate(-m_driverJoystick.getZ() * MaxAngularRate)
)
);

shooter.setDefaultCommand(
shooter.shootCommand(this::getRightTrigger, false)
);

operatorController.a().whileTrue(
shooter.shootCommand(() -> 0.5, true)
);

final var idle = new SwerveRequest.Idle();

RobotModeTriggers.disabled().whileTrue(
drivetrain.applyRequest(() -> idle).ignoringDisable(true)
);

// Reset gyro
new JoystickButton(m_driverJoystick, 2)
.onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
}

public Command getAutonomousCommand() {

final var idle = new SwerveRequest.Idle();

return Commands.sequence(

drivetrain.runOnce(() ->
drivetrain.seedFieldCentric(Rotation2d.kZero)
),

drivetrain.applyRequest(() ->
drive.withVelocityX(0.5)
.withVelocityY(0)
.withRotationalRate(0)
).withTimeout(5.0),

drivetrain.applyRequest(() -> idle)
);
}

public double getRightTrigger() {
double value = operatorController.getRightTriggerAxis();
return value < 0.1 ? 0 : value;
}
}
