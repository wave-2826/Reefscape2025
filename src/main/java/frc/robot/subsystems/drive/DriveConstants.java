package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;

public class DriveConstants {
    public record SwerveModuleConfiguration(int driveMotorCanID, int turnMotorCanID, Rotation2d zeroOffset) {
    }

    // TODO: Measure our effective maximum speed once robot is built
    public static final double maxSpeedMetersPerSec = 4.8; // 2.0;
    public static final double odometryFrequency = 100.0; // Hz
    public static final double trackWidth = Units.inchesToMeters(23.5);
    public static final double wheelBase = Units.inchesToMeters(23.5);
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final Translation2d[] moduleTranslations = new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0), new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
    };

    public static final int pigeonCanId = 10;

    // TODO: Measure module offsets for real drive base
    public static final SwerveModuleConfiguration frontLeftModule = new SwerveModuleConfiguration(41, 42,
        Rotation2d.fromDegrees(0.));
    public static final SwerveModuleConfiguration frontRightModule = new SwerveModuleConfiguration(11, 12,
        Rotation2d.fromDegrees(0.));
    public static final SwerveModuleConfiguration backLeftModule = new SwerveModuleConfiguration(21, 22,
        Rotation2d.fromDegrees(0.));
    public static final SwerveModuleConfiguration backRightModule = new SwerveModuleConfiguration(31, 32,
        Rotation2d.fromDegrees(0.));

    public static final boolean USE_SETPOINT_GENERATOR = true;

    // Drive motor configuration
    public static final int driveMotorCurrentLimit = 30;
    public static final double wheelRadiusMeters = Units.inchesToMeters(2);
    public static final double driveMotorReduction = Mk4Reductions.L2.reduction;
    public static final DCMotor driveSimMotor = DCMotor.getNeoVortex(1);

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
    public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

    // Drive PID configuration
    public static final double driveKp = 0.0;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.0;
    public static final double driveKv = 0.1;

    public static final double driveSimP = 0.6;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.12;

    // Turn motor configuration
    public static final boolean turnInverted = false;
    public static final int turnMotorCurrentLimit = 15;
    public static final double turnMotorReduction = Mk4Reductions.Turn.reduction;
    public static final DCMotor turnSimMotor = DCMotor.getNeoVortex(1);
    public static final AngularVelocity maxSteerVelocity = RadiansPerSecond.of(100);

    // Turn encoder configuration
    public static final boolean turnEncoderInverted = false;
    public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double turnAbsoluteEncoderPositionFactor = 2 * Math.PI;
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    // Turn PID configuration
    public static final double turnKp = 2.0;
    public static final double turnKd = 0.0;

    public static final double turnSimP = 13.0;
    public static final double turnSimD = 0.0;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

    // PathPlanner configuration
    public static final double robotMassKg = Units.lbsToKilograms(100.); // TODO: Update for real robot
    public static final double robotMOI = 6.883; // TODO: Measure for real robot
    public static final double wheelCOF = 0.9; // TODO: Measure for TPU wheels

    public static final RobotConfig pathplannerConfig = new RobotConfig(robotMassKg, robotMOI,
        new ModuleConfig(wheelRadiusMeters, maxSpeedMetersPerSec, wheelCOF,
            driveSimMotor.withReduction(driveMotorReduction), driveMotorCurrentLimit, 1),
        moduleTranslations);

    public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
        .withCustomModuleTranslations(moduleTranslations).withRobotMass(Kilogram.of(robotMassKg))
        .withGyro(COTS.ofPigeon2()).withSwerveModule(
            new SwerveModuleSimulationConfig(driveSimMotor, turnSimMotor, driveMotorReduction, turnMotorReduction,
                Volts.of(0.1), Volts.of(0.1), Meters.of(wheelRadiusMeters), KilogramSquareMeters.of(0.02), wheelCOF));

    private enum Mk4Reductions {
        // @formatter:off
        L1((50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0)),
        L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
        L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),
        L4((48.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0)),
        Turn((150.0 / 7.0));
        // @formatter:on

        final double reduction;

        Mk4Reductions(double reduction) {
            this.reduction = reduction;
        }
    }
}
