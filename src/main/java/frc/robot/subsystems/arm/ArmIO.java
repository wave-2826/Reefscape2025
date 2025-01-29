package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        /**
         * The distance reported by a range finder pointing at the game piece, allowing us to sense its presence.
         */
        public double gamePieceDistance = 0.0;
        public boolean elevatorLimitSwitchActive = false;
    }

    public default void updateInputs(ArmIOInputs inputs) {
    }
}
