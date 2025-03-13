package frc.robot.subsystems.pieceVision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface PieceVisionIO {
    @AutoLog
    public static class PieceVisionIOInputs {
        public boolean connected = false;

        public double framerate = 0;
        public double cpuTemp = 0;

        /**
         * The latest piece location observations. Can be null. Only set on frames that we receive a new observation.
         */
        public PieceLocations locations = null;
    }

    public static record PieceLocations(double timestampSeconds, PieceLocation[] locations) {
    }

    public static record PieceLocation(Rotation2d theta, Rotation2d pitch, double area) {
    }

    /**
     * Sets whether the vision system is enabled. This is called with false when the robot is disabled to reduce the
     * temperature rise of the camera.
     */
    public default void setEnabled(boolean enabled) {
    }

    public default void updateInputs(PieceVisionIOInputs inputs) {
    }
}
