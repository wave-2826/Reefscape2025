package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * Visualization for game pieces in the robot during simulation. This probably doesn't need to be its own class, but
 * it's here in case we ever want to make handling of this more complicated.
 */
public class SimRobotGamePieceVisualization {
    /**
     * The transformation of the coral relative to the robot. If null, we assume that we don't have a game piece.
     */
    private static Transform3d coralTransformation = null;

    public static void setCoralTransform(Transform3d transform) {
        coralTransformation = transform;
    }

    public static void update(Pose2d drivetrainPose) {
        Pose3d drivetrainPose3d = new Pose3d(drivetrainPose.getTranslation().getX(),
            drivetrainPose.getTranslation().getY(), 0.0,
            new Rotation3d(0.0, 0.0, drivetrainPose.getRotation().getRadians()));

        if(coralTransformation == null) {
            Logger.recordOutput("FieldSimulation/RobotCoralPose", new Pose3d[] {});
        } else {
            Logger.recordOutput("FieldSimulation/RobotCoralPose", new Pose3d[] {
                drivetrainPose3d.plus(coralTransformation)
            });
        }

        coralTransformation = null;
    }
}
