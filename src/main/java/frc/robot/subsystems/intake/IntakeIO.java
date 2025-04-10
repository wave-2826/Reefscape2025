package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public boolean intakeSensorTriggered = false;
        public boolean middleSensorTriggered = false;
        public boolean endSensorTriggered = false;

        /** The pitch of the intake. 0 degrees is straight outward. */
        public Rotation2d intakePitch = Rotation2d.kZero;
        /** The speed of the intake wheels, in radians per second. */
        public double intakeWheelSpeed = 0;
        /** The speed of the transport wheels, in radians per second. */
        public double transportWheelSpeed = 0;

        public boolean transportMotorConnected = false;
        public boolean intakePowerMotorConnected = false;
        public boolean intakePitchMotorConnected = false;
    }

    public default void updateInputs(IntakeIOInputs inputs) {
    }

    public default void setIntakePitch(Rotation2d pitch) {
    }

    /** Sets the intake to stop running closed-loop position control until setIntakePitch is called again. */
    public default void setIntakeCoast() {
    }

    public default void runVelocity(double intakePower, double transportPower) {
    }

    public default void overridePitchPower(double power) {
    }
}
