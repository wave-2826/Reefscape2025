package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public double gamePieceDistance = 0.0;
        public boolean elevatorLimitSwitchActive = false;
    }

    public default void updateInputs(ArmIOInputs inputs) {
    }
}
