package frc.robot.subsystems.intake;

import frc.robot.util.LoggedTunableSparkPID;
import frc.robot.util.PIDConstants;

/**
 * Constants related to the intake subsystem.
 */
public class IntakeConstants {
    public static int intakePitchMotorId = 44;
    public static LoggedTunableSparkPID pitchPID = new LoggedTunableSparkPID("IntakePitch") // Position
        .addRealRobotGains(new PIDConstants(0.4, 0, 0)).addSimGains(new PIDConstants(0.5, 0, 0));

    public static boolean pitchMotorInverted = false;
    public static boolean pitchEncoderInverted = true;

    public static int pitchMotorCurrentLimit = 30;

    public static double pitchAbsolutePositionFactor = 2 * Math.PI;
    public static double pitchAbsoluteVelocityFactor = pitchAbsolutePositionFactor / 60.;

    public static LoggedTunableSparkPID powerPID = new LoggedTunableSparkPID("IntakePower") // Velocity
        .addRealRobotGains(new PIDConstants(0.001, 0, 0, 1 / 6000.))
        .addSimGains(new PIDConstants(0.001, 0, 0, 1 / 6000.));

    public static double powerPositionConversionFactor = 2 * Math.PI;
    public static double powerVelocityConversionFactor = powerPositionConversionFactor / 60.;

    public static boolean powerMotorInverted = false;
    public static int powerMotorCurrentLimit = 65;

    public static double intakeZeroAngle = 0.0672255;

    public static int intakeDriveMotorId = 45;

    public static LoggedTunableSparkPID transportPID = new LoggedTunableSparkPID("Transport") // Velocity
        .addRealRobotGains(new PIDConstants(0.001, 0, 0, 1 / 6000.))
        .addSimGains(new PIDConstants(0.001, 0, 0, 1 / 6000.));

    public static int transportDriveMotorId = 46;
    public static int transportMotorCurrentLimit = 40;

    public static double transportPositionConversionFactor = 2 * Math.PI;
    public static double transportVelocityConversionFactor = transportPositionConversionFactor / 60.;

    public static boolean transportMotorInverted = false;

    public static int intakeSensorDIOPort = 0;
    public static int transportSensorDIOPort = 1;
}
