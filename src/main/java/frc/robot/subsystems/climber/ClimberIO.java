package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public boolean climberMotorConnected = false;
        public Rotation2d climberPosition = new Rotation2d();
    }

    public default void updateInputs(ClimberIOInputs inputs) {
    }

    public default void setClimberTargetAngle(Rotation2d angle) {
    }

    public default void setClimberBrakeMode(boolean enable) {
    }
}
