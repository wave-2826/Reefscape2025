package frc.robot.subsystems.pieceVision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

public interface PieceVisionIO {
    @AutoLog
    public static class PieceVisionIOInputs {
        public boolean connected = false;
        public Rotation2d theta = new Rotation2d();
    }

    public default void updateInputs(VisionIOInputs inputs) {
    }
}
