package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public boolean driveConnected = false;
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public boolean turnConnected = false;
        public boolean turnEncoderConnected = false;

        public Rotation2d relativeTurnPosition = new Rotation2d();
        public Rotation2d absoluteTurnPosition = new Rotation2d();
        public Rotation2d offsetAbsoluteTurnPosition = new Rotation2d();
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ModuleIOInputs inputs) {
    }

    /** Run the drive motor at the specified open loop value. */
    public default void setDriveOpenLoop(double output) {
    }

    /** Run the turn motor at the specified open loop value. */
    public default void setTurnOpenLoop(double output) {
    }

    /** Run the drive motor at the specified velocity. */
    public default void setDriveVelocity(double velocityRadPerSec, double feedforward) {
    }

    /** Run the turn motor to the specified rotation. */
    public default void setTurnPosition(Rotation2d rotation) {
    }

    /** Sets if the drive motor brake is enabled. */
    public default void setDriveBrakeMode(boolean enable) {
    }

    /** Sets if the turn motor brake is enabled. */
    public default void setTurnBrakeMode(boolean enable) {
    }

    /** Set P, I, and D gains for closed loop control on drive motor. */
    public default void setDrivePID(double kP, double kI, double kD) {
    }

    /** Resets the relative encoder to absolute if one is used. */
    public default void resetToAbsolute() {
    }

    /**
     * Set P gain, I gain, D gain, and derivative filter for closed loop control on turn motor.
     */
    public default void setTurnPID(double kP, double kI, double kD, double derivativeFilter) {
    }

    /** Temporarily change the drive motor current limit. */
    public default void setDriveCurrentLimit(int limitAmps) {
    }
}
