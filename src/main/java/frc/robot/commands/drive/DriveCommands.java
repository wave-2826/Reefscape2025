package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
    private static final double DEADBAND = 0.1;
    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;

    private DriveCommands() {
    }

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public static Command joystickDrive(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
        DoubleSupplier omegaSupplier) {
        return Commands.run(() -> {
            // Get linear velocity
            Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
                ySupplier.getAsDouble());

            // Apply rotation deadband
            double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

            // Square rotation value for more precise control
            omega = Math.copySign(omega * omega, omega);

            // Convert to field relative speeds & send command
            ChassisSpeeds speeds = new ChassisSpeeds(linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                omega * drive.getMaxAngularSpeedRadPerSec());
            boolean isFlipped = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
                isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
        }, drive);
    }

    /** Returns a command that drives straight at the specified speed. Positive numbers are forward. */
    public static Command driveStraightCommand(Drive drive, double speedMetersPerSecond) {
        return Commands.run(() -> {
            drive.runVelocity(new ChassisSpeeds(speedMetersPerSecond, 0., 0.));
        }, drive);
    }

    /**
     * Returns a command that drives straight at the specified speed for the specified duration. Positive numbers are
     * forward.
     */
    public static Command driveStraightCommand(Drive drive, double speedMetersPerSecond, double timeSeconds) {
        return driveStraightCommand(drive, speedMetersPerSecond).raceWith(Commands.waitSeconds(timeSeconds));
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control. Possible use cases
     * include snapping to an angle, aiming at a vision target, or controlling absolute rotation with a joystick.
     */
    public static Command joystickDriveAtAngle(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
        Supplier<Rotation2d> rotationSupplier) {
        // Create PID controller
        ProfiledPIDController angleController = new ProfiledPIDController(ANGLE_KP, 0.0, ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return Commands.run(() -> {
            // Get linear velocity
            Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
                ySupplier.getAsDouble());

            // Calculate angular speed
            double omega = angleController.calculate(drive.getRotation().getRadians(),
                rotationSupplier.get().getRadians());

            // Convert to field relative speeds & send command
            ChassisSpeeds speeds = new ChassisSpeeds(linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(), omega);
            boolean isFlipped = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
                isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
        }, drive).beforeStarting( // Reset PID controller when command starts
            () -> angleController.reset(drive.getRotation().getRadians()));
    }
}
