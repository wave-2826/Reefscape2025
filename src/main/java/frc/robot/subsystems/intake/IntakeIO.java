package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        boolean intakeSensorTriggered = false;
        boolean pieceMovingSensorTriggered = false;
        boolean pieceSeatedSensorTriggered = false;
    }

    public default void updateInputs(ArmIOInputs inputs) {
    }
}
