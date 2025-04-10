package frc.robot.subsystems.pieceVision;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.FieldConstants;

public class PieceVisionIOSim implements PieceVisionIO {
    /** The framerate of the simulated camera, in FPS. */
    private static final double FRAMERATE = 18.;
    /** The amount of variance of the simulated framerate, in FPS. */
    private static final double FRAMERATE_VARIANCE = 0.5;
    /** The horizontal FOV of the simulated camera. */
    private static final Rotation2d HORIZONTAL_FOV = Rotation2d.fromDegrees(63.3);
    /** The vertical FOV of the simulated camera. */
    private static final Rotation2d VERTICAL_FOV = Rotation2d.fromDegrees(49.7);
    /**
     * The horizontal distance before which the camera will not track pieces, in meters. This matches where the pieces
     * are occluded by the intake.
     */
    private static final double MIN_DISTANCE = Units.inchesToMeters(5 + 15);
    /**
     * The horizontal distance before which the camera will not track pieces, in meters.
     */
    private static final double MAX_DISTANCE = FieldConstants.fieldLength / 3.;
    /**
     * The chance each frame for a piece to randomly not be registered. Scaled by the distance in meters.
     */
    private static final double FAILURE_CHANCE = 0.05;

    /** The maximnmum deviation of recorded piece positions. */
    private static final Rotation2d MAX_DEVIATION = Rotation2d.fromDegrees(0.25);

    private final Timer frameTimer = new Timer();
    private double nextFrameTime = 1. / FRAMERATE;

    private Supplier<Pose2d> robotPoseSupplier;

    public PieceVisionIOSim(Supplier<Pose2d> robotPoseSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
        frameTimer.start();
    }

    private boolean shouldRenderFrame() {
        boolean advanced = frameTimer.advanceIfElapsed(nextFrameTime);
        if(advanced) {
            nextFrameTime = 1. / (FRAMERATE + (Math.random() * FRAMERATE_VARIANCE - FRAMERATE_VARIANCE / 2.));
        }
        return advanced;
    }

    private PieceLocation[] getPieceLocations() {
        var gamePieceLocations = SimulatedArena.getInstance().getGamePiecesArrayByType("Coral");
        List<PieceLocation> locations = new ArrayList<>();

        Pose2d robotPose = robotPoseSupplier.get();
        Pose3d robotPose3d = new Pose3d(robotPose.getTranslation().getX(), robotPose.getTranslation().getY(), 0.0,
            new Rotation3d(0.0, 0.0, robotPose.getRotation().getRadians()));
        Pose3d cameraPose = robotPose3d.plus(PieceVisionConstants.robotToCamera);
        Logger.recordOutput("PieceVision/Camera", cameraPose);

        for(var pieceLocation : gamePieceLocations) {
            Pose3d robotToPiece = pieceLocation.relativeTo(robotPose3d);
            double pieceDistance = -robotToPiece.getTranslation().getX();
            if(pieceDistance < MIN_DISTANCE) {
                continue;
            }
            if(pieceDistance > MAX_DISTANCE) {
                continue;
            }

            double failureChance = Math.min(0.9,
                Math.max(0., Math.pow((pieceDistance - MIN_DISTANCE), 2) * FAILURE_CHANCE));
            if(Math.random() < failureChance) {
                continue;
            }

            Translation3d cameraToPiece = pieceLocation.relativeTo(cameraPose).getTranslation();
            double theta = Math.atan2(cameraToPiece.getY(), cameraToPiece.getX())
                + (Math.random() - 0.5) * MAX_DEVIATION.getRadians();
            double pitch = Math.atan2(cameraToPiece.getZ(), cameraToPiece.toTranslation2d().getNorm())
                + (Math.random() - 0.5) * MAX_DEVIATION.getRadians();

            if(Math.abs(theta) < HORIZONTAL_FOV.getRadians() / 2. && Math.abs(pitch) < VERTICAL_FOV.getRadians() / 2.) {
                double area = Math.pow(Math.max(0.1, pieceDistance - MIN_DISTANCE), 0.5) * 0.05;
                locations.add(new PieceLocation(Rotation2d.fromRadians(theta), Rotation2d.fromRadians(pitch), area));
            }

            // Pose3d lineEnd = cameraPose
            //     .transformBy(new Transform3d(new Translation3d(), new Rotation3d(0, -pitch, theta)))
            //     .transformBy(new Transform3d(new Translation3d(10, 0, 0), Rotation3d.kZero));
            // Logger.recordOutput("PieceVision/Piece" + locations.size() + "Line", new Pose3d[] {
            //     cameraPose, lineEnd
            // });
        }

        return locations.toArray(new PieceLocation[0]);
    }

    @Override
    public void updateInputs(PieceVisionIOInputs inputs) {
        if(!shouldRenderFrame()) {
            inputs.locations = null;
            return;
        }

        inputs.connected = true;
        inputs.cpuTemp = 25;
        inputs.framerate = 1. / nextFrameTime;
        inputs.locations = getPieceLocations();
        inputs.timestamp = Timer.getTimestamp();
    }
}
