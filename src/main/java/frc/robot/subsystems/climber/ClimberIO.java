package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public boolean climberMotorConnected = false;
    }

    public default void updateInputs(ClimberIOInputs inputs) {
    }
}
