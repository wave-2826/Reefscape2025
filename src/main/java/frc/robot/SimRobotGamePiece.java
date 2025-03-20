package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * Visualization and tracking for game pieces in the robot during simulation. This probably doesn't need to be its own
 * class, but it's here in case we ever want to make handling of this more complicated.
 */
public class SimRobotGamePiece {
    /**
     * The transformation of the coral relative to the robot. If null, we assume that we don't have a game piece.
     */
    private static Transform3d coralTransformation = null;
    /**
     * The latest transformation of the coral relative to the robot, not nulled every update.
     */
    private static Transform3d latestCoralTransformation = null;

    public static void setCoralTransform(Transform3d transform) {
        coralTransformation = transform;
        latestCoralTransformation = transform;
    }

    public static Transform3d getCoralTransform() {
        return latestCoralTransformation;
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
