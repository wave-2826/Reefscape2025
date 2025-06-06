package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.Container;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
    private static final double DEADBAND = 0.15;
    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;

    private DriveCommands() {
    }

    public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero)).getTranslation();
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public static Command joystickDrive(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
        DoubleSupplier omegaSupplier) {
        var robotState = RobotState.getInstance();
        return Commands.run(() -> {
            // Get linear velocity
            Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
                ySupplier.getAsDouble());

            // Apply rotation deadband
            double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

            // Square rotation value for more precise control
            omega = Math.copySign(omega * omega, omega);

            // Convert to field relative speeds & send command
            ChassisSpeeds speeds = new ChassisSpeeds(linearVelocity.getX() * DriveConstants.maxSpeedMetersPerSec,
                linearVelocity.getY() * DriveConstants.maxSpeedMetersPerSec,
                omega * DriveConstants.maxAngularSpeedRadPerSec);
            boolean isFlipped = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
                isFlipped ? robotState.getRotation().plus(new Rotation2d(Math.PI)) : robotState.getRotation()));
        }, drive);
    }

    /** Returns a command that drives straight at the specified speed. Positive numbers are forward. */
    public static Command driveStraightCommand(Drive drive, double speedMetersPerSecond,
        Supplier<Rotation2d> fieldAngle, Supplier<Rotation2d> robotAngle) {
        Container<Rotation2d> fieldAngleTarget = new Container<>(Rotation2d.kZero);

        var robotState = RobotState.getInstance();
        try(PIDController thetaController = new PIDController(6.5, 0.0, 0.3)) {
            thetaController.enableContinuousInput(0, Math.PI * 2);
            return Commands.sequence(Commands.runOnce(() -> {
                thetaController.reset();
                fieldAngleTarget.value = fieldAngle.get();
                if(robotAngle != null) thetaController.setSetpoint(robotAngle.get().getRadians());
            }), Commands.run(() -> {
                double thetaSpeed = robotAngle == null ? 0.
                    : thetaController.calculate(robotState.getRotation().getRadians());

                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(fieldAngleTarget.value.getCos() * speedMetersPerSecond,
                        fieldAngleTarget.value.getSin() * speedMetersPerSecond, thetaSpeed, robotState.getRotation()));
            }, drive)).withName("DriveStraight").finallyDo(drive::stop);
        }
    }

    /**
     * Returns a command that drives straight at the specified speed until stopped. Positive numbers are forward.
     */
    public static Command driveStraightCommand(Drive drive, double speedMetersPerSecond,
        Supplier<Rotation2d> robotAngle) {
        return driveStraightCommand(drive, speedMetersPerSecond, RobotState.getInstance()::getRotation, robotAngle);
    }

    /**
     * Returns a command that drives straight at the specified speed for the specified duration. Positive numbers are
     * forward.
     */
    public static Command driveStraightCommand(Drive drive, double speedMetersPerSecond, double timeSeconds,
        Supplier<Rotation2d> robotAngle) {
        return driveStraightCommand(drive, speedMetersPerSecond, RobotState.getInstance()::getRotation, robotAngle)
            .withTimeout(timeSeconds);
    }

    /**
     * Returns a command that drives at the specified speed and angle for the specified duration. Positive numbers are
     * forward.
     */
    public static Command driveStraightCommand(Drive drive, double speedMetersPerSecond, double timeSeconds,
        Supplier<Rotation2d> fieldAngle, Supplier<Rotation2d> robotAngle) {
        return driveStraightCommand(drive, speedMetersPerSecond, fieldAngle, robotAngle).withTimeout(timeSeconds);
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

        var robotState = RobotState.getInstance();
        return Commands.run(() -> {
            // Get linear velocity
            Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
                ySupplier.getAsDouble());

            // Calculate angular speed
            double omega = angleController.calculate(robotState.getRotation().getRadians(),
                rotationSupplier.get().getRadians());

            // Convert to field relative speeds & send command
            ChassisSpeeds speeds = new ChassisSpeeds(linearVelocity.getX() * DriveConstants.maxSpeedMetersPerSec,
                linearVelocity.getY() * DriveConstants.maxSpeedMetersPerSec, omega);
            boolean isFlipped = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
                isFlipped ? robotState.getRotation().plus(new Rotation2d(Math.PI)) : robotState.getRotation()));
        }, drive).beforeStarting( // Reset PID controller when command starts
            () -> angleController.reset(robotState.getRotation().getRadians()));
    }
}
