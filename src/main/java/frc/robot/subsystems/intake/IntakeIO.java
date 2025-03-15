package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        boolean intakeSensorTriggered = false;
        boolean transortSensorTriggered = false;
    }

    public default void updateInputs(IntakeIOInputs inputs) {
    }

    public default void setIntakePitch(Rotation2d pitch) {
    }

    public default void runIntakeOpenLoop(double power) {
    }
}
