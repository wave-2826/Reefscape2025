package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.ClosedLoopConfig;

/**
 * Constants related to the intake subsystem.
 */
public class IntakeConstants {
    public static int frontBottomMotorId = 44;
    public static ClosedLoopConfig frontBottomClosedLoopConfig = new ClosedLoopConfig() // Velocity
        .pid(/* TODO */ 0.5, 0.0, 0.0);

    public static int frontTopMotorId = 45;
    public static ClosedLoopConfig frontTopClosedLoopConfig = new ClosedLoopConfig() // Velocity
        .pid(/* TODO */ 0.5, 0.0, 0.0);

    public static int agitatorMotor1Id = 46;
    public static int agitatorMotor2Id = 47;
    public static ClosedLoopConfig agitatorClosedLoopConfig = new ClosedLoopConfig() // Velocity
        .pid(/* TODO */ 0.5, 0.0, 0.0);

    public static int beltMotorId = 48;
    public static ClosedLoopConfig beltClosedLoopConfig = new ClosedLoopConfig() // Velocity
        .pid(/* TODO */ 0.5, 0.0, 0.0);
}
