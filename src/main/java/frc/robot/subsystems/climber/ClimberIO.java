package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public boolean climberMotorConnected = false;
        public Rotation2d climberAbsolutePosition = Rotation2d.kZero;
    }

    /**
     * Reset the climber encoder to the absolute encoder's position. We don't use closed-loop control directly with the
     * absolute encoder because otherwise we can get stuck turning the motor into an invalid state if the climber arm is
     * resting on an object (e.g. the cage...).
     */
    public default void resetToAbsolute() {
    }

    public default void updateInputs(ClimberIOInputs inputs) {
    }

    public default void setClimberTargetAngle(Rotation2d angle) {
    }

    public default void runClimberOpenLoop(double power) {
    }

    public default void setClimberBrakeMode(boolean enable) {
    }
}
