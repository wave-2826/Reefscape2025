package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.arm.ArmState.WristRotation;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public boolean gamePiecePresent = false;

        /** The recorded arm pitch position. 0 degrees is upward and positive is clockwise. */
        public Rotation2d armPitchPosition = Rotation2d.kZero;
        /** The recorded arm pitch angular velocity in radians per second. */
        public double armPitchVelocity = 0.0;

        /** The recorded arm wrist position. */
        public Rotation2d armWristPosition = Rotation2d.kZero;
        /** The recorded arm wrist velocity in radians per second. */
        public double armWristVelocity = 0.0;

        /** The end effector velocity in radians per second. Positive velocity is outward. */
        public double endEffectorVelocity = 0.0;

        /** The relative height of the elevator in meters. */
        public double elevatorHeightMeters = 0.;
        /** The elevator velocity in meters per second. */
        public double elevatorVelocityMetersPerSecond = 0.;

        /** The last measured height from the absolute height sensor. */
        public double absoluteHeightMeters = 0.;
        /** If our last absolute height sensor measurement was valid. */
        public boolean validAbsoluteMeasurement = false;

        public boolean elevatorMotorsConnected = false;
        public boolean elevatorHeightSensorConnected = false;
        public boolean armPitchMotorConnected = false;
        public boolean armWristMotorConnected = false;
        public boolean endEffectorMotorConnected = false;
    }

    /**
     * Resets the elevator height to the absolutely measured value.
     */
    public default void resetToAbsolute() {
    }

    /**
     * Reset the elevator to the bottom-most position it can be in. Used for manually resetting the elevator if the
     * height sensor malfunctions.
     */
    public default void resetToBottom() {
    }

    public default void setElevatorHeight(double heightMeters, double feedforwardVolts) {
    }

    public default void setArmPitchPosition(Rotation2d position, double feedforward) {
    }

    public default void setWristRotation(WristRotation rotation) {
    }

    public default void setEndEffectorState(EndEffectorState mode) {
    }

    public default void overrideHeightPower(double power, double feedforward) {
    }

    public default void overridePitchPower(double power) {
    }

    public default void overrideWristPower(double power) {
    }

    public default void overrideEndEffectorPower(double power) {
    }

    public default void updateInputs(ArmIOInputs inputs) {
    }
}
