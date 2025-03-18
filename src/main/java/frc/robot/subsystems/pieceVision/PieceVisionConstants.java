package frc.robot.subsystems.pieceVision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class PieceVisionConstants {
    /** The transform of the piece vision camera. */
    public static final Transform3d cameraTransform = new Transform3d(new Translation3d(//
        Units.inchesToMeters(1.0), // Forward is +X
        Units.inchesToMeters(0.0), // Left is +Y
        Units.inchesToMeters(33.5) // Up is +Z
    ), // TODO
        new Rotation3d(0, Units.degreesToRadians(15.), 0).rotateBy(new Rotation3d(0, 0, Math.PI)));

    /** The hostname of the camera. */
    public static final String cameraHostname = "limelight";
}
