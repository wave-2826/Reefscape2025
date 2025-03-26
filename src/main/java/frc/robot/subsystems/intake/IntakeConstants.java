package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.LoggedTunableSparkPID;
import frc.robot.util.PIDConstants;

/**
 * Constants related to the intake subsystem.
 */
public class IntakeConstants {
    public static final int intakePitchMotorId = 44;
    public static final LoggedTunableSparkPID pitchPID = new LoggedTunableSparkPID("Intake/Pitch") // Position
        .addRealRobotGains(new PIDConstants(0.5, 0, 0.4)).addSimGains(new PIDConstants(5.0, 0, 0));

    public static final DCMotor intakePitchMotor = DCMotor.getNeo550(1);

    public static final boolean pitchMotorInverted = true;
    public static final boolean pitchEncoderInverted = false;

    public static final int pitchMotorCurrentLimit = 40;

    public static final double pitchAbsolutePositionFactor = 2 * Math.PI;
    public static final double pitchAbsoluteVelocityFactor = pitchAbsolutePositionFactor / 60.;

    public static final LoggedTunableSparkPID powerPID = new LoggedTunableSparkPID("Intake/Power") // Velocity
        .addRealRobotGains(new PIDConstants(0.001, 0, 0, 1 / 6000.))
        .addSimGains(new PIDConstants(0.001, 0, 0, 1 / 6000.));

    public static final DCMotor intakePowerMotor = DCMotor.getNeoVortex(1);

    public static final double powerPositionConversionFactor = 2 * Math.PI;
    public static final double powerVelocityConversionFactor = powerPositionConversionFactor / 60.;

    public static final boolean powerMotorInverted = false;
    public static final int powerMotorCurrentLimit = 65;

    public static final double intakeZeroAngle = 0.9921534;

    public static final int intakeDriveMotorId = 45;

    public static final LoggedTunableSparkPID transportPID = new LoggedTunableSparkPID("Transport") // Velocity
        .addRealRobotGains(new PIDConstants(0.001, 0, 0, 1 / 6000.))
        .addSimGains(new PIDConstants(0.001, 0, 0, 1 / 6000.));

    public static final DCMotor transportMotor = DCMotor.getNeo550(1);

    public static final int transportDriveMotorId = 46;
    public static final int transportMotorCurrentLimit = 40;

    public static final double transportPositionConversionFactor = 2 * Math.PI;
    public static final double transportVelocityConversionFactor = transportPositionConversionFactor / 60.;

    public static final boolean transportMotorInverted = false;

    public static final int intakeSensorDIOPort = 0;
    public static final int middleSensorDIOPort = 2;
    public static final int endSensorDIOPort = 1;

    // For simulation
    public static final Distance intakeLength = Inches.of(17);
    public static final Translation3d intakeOrigin = new Translation3d(-0.229, 0.002, 0.196);
}
