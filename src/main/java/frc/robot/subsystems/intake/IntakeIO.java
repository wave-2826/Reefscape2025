package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public boolean intakeSensorTriggered = false;
        public boolean transortSensorTriggered = false;
        /** The pitch of the intake. 0 degrees is straight outward. */
        public Rotation2d intakePitch = Rotation2d.kZero;
        /** The speed of the intake wheels, in radians per second. */
        public double intakeWheelSpeed = 0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {
    }

    public default void setIntakePitch(Rotation2d pitch) {
    }

    public default void setIntakeCoast() {
    }

    public default void runVelocity(double intakePower, double transportPower) {
    }

    public default void overridePitchPower(double power) {
    }
}
