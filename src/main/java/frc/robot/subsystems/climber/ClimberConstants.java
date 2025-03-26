package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.LoggedTunableSparkPID;
import frc.robot.util.PIDConstants;

/**
 * Constants related to the climber subsystem.
 */
public class ClimberConstants {
    public static final int climberMotorId = 36;
    /** The climber encoder zero angle, in radians. */
    public static final double climberZeroAngle = 0.5668083;
    public static final boolean invertClimber = false;
    public static final LoggedTunableSparkPID climberPID = new LoggedTunableSparkPID("Climber")
        .addRealRobotGains(new PIDConstants(6.0, 0.0, 2.0)).addSimGains(new PIDConstants(0.5, 0.0, 0.0));

    /** The climber motor current limit in amps. */
    public static final int climberMotorCurrentLimit = 40;
    /** Whether the climber motor is inverted. */
    public static final boolean climberMotorInverted = false;

    /** The position conversion factor from absolute encoder rotations to climber radians. */
    public static final double climberAbsolutePositionFactor = 2 * Math.PI;
    /** The velocity conversion factor from absolute encoder RPM to climber radians per second. */
    public static final double climberAbsoluteVelocityFactor = climberAbsolutePositionFactor / 60.0;

    /** The climber reduction. */
    public static final double climberReduction = 75. * 2.;
    /** The position on the climber arm that the strap is at, in meters. */
    public static final double climberArmStrapPosition = Units.inchesToMeters(5.5);
    /** The radius of the climber pulley, in meters. */
    public static final double climberPulleyRadius = Units.inchesToMeters(1.2 / 2.);
    /** The length of the triangle leg from the climber pulley center to the climber pivot center, in meters. */
    public static final double climberPulleyToPivotDistance = Units.inchesToMeters(6.5);
    /** The inclanation of the triangle leg from the... whatever you get it, in radians */
    public static final double climberLegAngle = Units.degreesToRadians(30.);
    /** The length of the climber strap in the resting position, in meters. */
    public static final double climberRestingLength = Units.inchesToMeters(10.5);

    /** The position conversion factor from motor encoder rotations to climber radians. */
    public static final double climberPositionConversionFactor = 2 * Math.PI / climberReduction;
    /** The velocity conversion factor from motor encoder RPM to climber radians per second. */
    public static final double climberVelocityConversionFactor = climberPositionConversionFactor / 60.0;

    public static final Translation3d climberOrigin = new Translation3d(0, 0.356, 0.196);
    public static final Distance climberArmLength = Meters.of(0.3);

    // For simulation
    public static final DCMotor climberMotor = DCMotor.getNEO(1);
}
