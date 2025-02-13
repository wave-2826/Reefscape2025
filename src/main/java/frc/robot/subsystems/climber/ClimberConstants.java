package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.LoggedTunableSparkPID;

/**
 * Constants related to the climber subsystem.
 */
public class ClimberConstants {
    public static final int climberMotorId = /* TODO */ 36;
    public static final Rotation2d climberZeroAngle = Rotation2d.fromDegrees(105.30);
    public static final boolean invertClimber = false;
    public static final LoggedTunableSparkPID climberPID = new LoggedTunableSparkPID("Climber")
        .addRealRobotGains(0.5, 0.0, 0.0).addSimGains(0.5, 0.0, 0.0);
}
