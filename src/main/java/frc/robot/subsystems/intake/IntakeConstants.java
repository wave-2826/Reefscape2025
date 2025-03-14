package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.ClosedLoopConfig;

/**
 * Constants related to the intake subsystem.
 */
public class IntakeConstants {
    public static int intakePitchMotorId = 44;
    public static ClosedLoopConfig intakePitchClosedLoopConfig = new ClosedLoopConfig() // Velocity
        .pid(/* TODO */ 0.5, 0.0, 0.0);

    public static int intakeDriveMotorId = 45;
    public static ClosedLoopConfig intakeDriveClosedLoopConfig = new ClosedLoopConfig() // Velocity
        .pid(/* TODO */ 0.5, 0.0, 0.0);

    public static int transportDriveMotorId = 46;
    public static ClosedLoopConfig transportClosedLoopConfig = new ClosedLoopConfig() // Velocity
        .pid(/* TODO */ 0.5, 0.0, 0.0);
}
