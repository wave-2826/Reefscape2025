package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public boolean gamePiecePresent = false;

        public boolean elevatorLimitSwitchActive = false;

        /** The recorded arm pitch position. */
        public Rotation2d armPitchPosition = new Rotation2d();
        /** The recorded arm pitch angular velocity in radians per second. */
        public double armPitchVelocity = 0.0;

        /** The recorded arm wrist position. */
        public Rotation2d armWristPosition = new Rotation2d();
        /** The recorded arm wrist velocity in radians per second. */
        public double armWristVelocity = 0.0;

        /** The end effector velocity in radians per second. */
        public double endEffectorVelocity = 0.0;
    }

    public default void updateElevatorPosition() {
    }

    public default void updateInputs(ArmIOInputs inputs) {
    }
}
