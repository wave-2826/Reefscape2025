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
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;

public class DriveConstants {
    public record SwerveModuleConfiguration(int driveMotorCanID, int turnMotorCanID, Rotation2d zeroOffset) {
    }

    public static final double maxSpeedMetersPerSec = 3.7; // "Magic" number from max speed measurement
    public static final double odometryFrequency = 100.0; // Hz
    public static final double bumperSizeMeters = Units.inchesToMeters(37.625);
    public static final double trackWidth = Units.inchesToMeters(24.);
    public static final double wheelBase = Units.inchesToMeters(24.);
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final Translation2d[] moduleTranslations = new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0), new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
    };

    public static final int pigeonCanId = 10;

    // The "front" of the robot is the elevator side. Yes, "front" technically doesn't mean anything,
    // but it makes it much easier to refer to locations.

    // TODO: Command to tune wheel offsets
    public static final SwerveModuleConfiguration frontLeftModule = new SwerveModuleConfiguration(41, 42,
        Rotation2d.fromRadians(1.219));
    public static final SwerveModuleConfiguration frontRightModule = new SwerveModuleConfiguration(11, 12,
        Rotation2d.fromRadians(1.407));
    public static final SwerveModuleConfiguration backLeftModule = new SwerveModuleConfiguration(21, 22,
        Rotation2d.fromRadians(5.973));
    public static final SwerveModuleConfiguration backRightModule = new SwerveModuleConfiguration(31, 32,
        Rotation2d.fromRadians(3.983));

    public static final boolean USE_SETPOINT_GENERATOR = false;

    // Drive motor configuration
    public static final int driveMotorCurrentLimit = 70;
    public static final double wheelRadiusMeters = Units.inchesToMeters(1.962); // "Magic" number from wheel radius characterization
    public static final double driveMotorReduction = Mk4Reductions.L2.reduction;
    public static final DCMotor driveSimMotor = DCMotor.getNeoVortex(1);

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
    public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

    // Drive PID configuration
    public static final double driveKp = 0.00022524;
    public static final double driveKd = 0.0;
    /** The static feedforward gain in volts. */
    public static final double driveKs = 0.17179; // "Magic" number from SysID
    /** The velocity gain in volts per (radian per second of wheel) */
    public static final double driveKv = 0.11; // "Magic" number from SysID
    /** The acceleration gain in volts per (radian per second per second of wheel) */
    public static final double driveKa = 0.024772;

    public static final double driveSimP = 0.6;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0311; // "Magic" number from SysID
    public static final double driveSimKv = 0.1344; // "Magic" number from SysID
    public static final double driveSimKt = driveMotorReduction / driveSimMotor.KtNMPerAmp;
    public static final double driveSimKa = 0.0225; // "Magic" number from SysID

    // Turn motor configuration
    public static final boolean turnInverted = false;
    public static final int turnMotorCurrentLimit = 35;
    public static final double turnMotorReduction = Mk4Reductions.Turn.reduction;
    public static final DCMotor turnSimMotor = DCMotor.getNeoVortex(1);
    public static final AngularVelocity maxSteerVelocity = RadiansPerSecond.of(100);

    // Turn encoder configuration
    public static final boolean turnEncoderInverted = false;
    public static final double turnEncoderPositionFactor = 2 * Math.PI / turnMotorReduction; // Motor rotations -> Radians
    public static final double turnAbsoluteEncoderPositionFactor = 2 * Math.PI / 5.; // Volts -> Radians
    public static final double turnEncoderVelocityFactor = turnEncoderPositionFactor / 60.0; // RPM -> Rad/Sec
    public static final double turnAbsoluteEncoderVelocityFactor = turnAbsoluteEncoderPositionFactor; // Volts/Sec -> Rad/Sec

    // The coupling factor between module rotation and drive wheel rotation.
    // In other words, 1 rotation of the azimuth results in turnDriveCouplingFactor drive wheel turns.
    // Because the serve modules are coaxial, the drive wheel's rotation is linked to the module rotation.
    // In practice, this has a very minimal effect on accuracy. However, it's pretty simple to account for,
    // so we do.
    // The coupling ratio is the inverse of the first stage of the drive gear ratio (which is the case for all current COTS modules).
    public static final double turnDriveCouplingFactor = 1. / Mk4Reductions.L2.firstStageReduction
        / driveMotorReduction;

    // Turn PID configuration
    public static final double turnKp = 2.5;
    public static final double turnKd = 2.0;
    public static final double turnDerivativeFilter = 0.0;

    public static final double turnSimP = 13.0;
    public static final double turnSimD = 0.0;
    public static final double turnPIDMinInput = -Math.PI; // Radians
    public static final double turnPIDMaxInput = Math.PI; // Radians

    // PathPlanner configuration
    public static final double robotMassKg = Units.lbsToKilograms(114.5 + 13 + 13);
    /**
     * The moment of inertia of the robot, in KG*M^2. Based on SysId:
     * https://pathplanner.dev/robot-config.html#calculating-moi-through-sysid-recommended
     * <p>
     * Currently roughly ~8.95.
     */
    public static final double robotMOIKgSqM = robotMassKg * driveBaseRadius * driveKa / 0.042656;
    public static final double wheelCOF = 1.048; // "Magic" number from slip current measurement

    public static final RobotConfig pathplannerConfig = new RobotConfig(robotMassKg, robotMOIKgSqM,
        new ModuleConfig(wheelRadiusMeters, maxSpeedMetersPerSec, wheelCOF,
            driveSimMotor.withReduction(driveMotorReduction), driveMotorCurrentLimit, 1),
        moduleTranslations);

    public static final PPHolonomicDriveController simHolonomicDriveController = new PPHolonomicDriveController(
        new PIDConstants(13.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0));
    public static final PPHolonomicDriveController realHolonomicDriveController = new PPHolonomicDriveController(
        new PIDConstants(6.5, 0.0, 0.25), new PIDConstants(8.0, 1.0, 0.75));

    public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
        .withCustomModuleTranslations(moduleTranslations).withRobotMass(Kilogram.of(robotMassKg))
        .withBumperSize(Meters.of(bumperSizeMeters), Meters.of(bumperSizeMeters)).withGyro(COTS.ofPigeon2())
        .withSwerveModule(
            new SwerveModuleSimulationConfig(driveSimMotor, turnSimMotor, driveMotorReduction, turnMotorReduction,
                Volts.of(0.1), Volts.of(0.1), Meters.of(wheelRadiusMeters), KilogramSquareMeters.of(0.02), wheelCOF));

    private enum Mk4Reductions {
        // @formatter:off
        L1(50.0 / 14.0, 19.0 / 25.0, 45.0 / 15.0),
        L2(50.0 / 14.0, 17.0 / 27.0, 45.0 / 15.0),
        L3(50.0 / 14.0, 16.0 / 28.0, 45.0 / 15.0),
        L4(48.0 / 16.0, 16.0 / 28.0, 45.0 / 15.0),
        Turn((12.8 / 1.0));
        // @formatter:on

        final double firstStageReduction;
        final double reduction;

        Mk4Reductions(double reduction) {
            this.firstStageReduction = reduction;
            this.reduction = reduction;
        }

        Mk4Reductions(double firstStage, double secondStage, double thirdStage) {
            this.reduction = firstStage * secondStage * thirdStage;
            this.firstStageReduction = firstStage;
        }
    }
}
