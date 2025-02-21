package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.LoggedTunableSparkPID;

/**
 * Constants related to the climber subsystem.
 */
public class ClimberConstants {
    public static final int climberMotorId = /* TODO */ 36;
    /** The climber encoder zero angle, in radians. */
    public static final double climberZeroAngle = 0.3248305;
    public static final boolean invertClimber = false;
    public static final LoggedTunableSparkPID climberPID = new LoggedTunableSparkPID("Climber")
        .addRealRobotGains(6.0, 0.0, 2.0).addSimGains(0.5, 0.0, 0.0);

    /** The climber motor current limit in amps. */
    public static final int climberMotorCurrentLimit = 40;
    /** Whether the climber motor is inverted. */
    public static final boolean climberMotorInverted = false;

    /** The position conversion factor from absolute encoder rotations to climber radians. */
    public static final double climberPositionConversionFactor = 2 * Math.PI;
    /** The velocity conversion factor from absolute encoder RPM to climber radians per second. */
    public static final double climberVelocityConversionFactor = climberPositionConversionFactor / 60.0;

    public static final Translation3d climberOrigin = new Translation3d(0, 0, 0); // TODO: Determine from CAD
    public static final Distance climberArmLength = Meters.of(0.5); // TODO: Determine from CAD
}
