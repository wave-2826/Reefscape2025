package frc.robot.subsystems.pieceVision;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.pieceVision.PieceVisionIO.PieceLocation;
import frc.robot.subsystems.pieceVision.PieceVisionIO.PieceLocations;
import frc.robot.util.LoggedTracer;

/**
 * Handles autonomous tracking of game pieces. This is separated from the main vision subsystem because it doesn't need
 * to interact and it makes the code cleaner.
 * 
 * If we have a new observation, continue tracking a piece. Our tracking system needs to be pretty sophisticated to
 * handle the robot moving, piece moving, pieces flickering in and out, and multiple pieces simultaneously. This leaves
 * a ton of possible edge cases to handle, so we need to be very careful. Our algorithm works like this:
 * 
 * <ol>
 * <li>If we don't have a new observation, do nothing.</li>
 * <li>If we're not currently "locked on" to a piece, pick the best piece to track. We currently determine this by the
 * piece with the largest area. However, we discard pieces that are too large (>50% of the screen). This may not be
 * necessary, but it's an additional safeguard against the neural network detecting something that isn't a piece.</li>
 * <li>If we are currently locked on to a piece, perform the following:
 * <ul>
 * <li>If there isn't a piece within a certain number of degrees of our last observation, we've likely lost the piece.
 * In this case, we wait a set amount of time to see if the piece comes back into view. If it doesn't, we try to find a
 * new piece to track by clearing our locked piece.</li>
 * <li>If the piece is near our last observation, we update our locked piece with the new observation.</li>
 * </ul>
 * </li>
 * <li>If we have a locked piece, we create a line from where our robot was when the observation occured to the piece.
 * We don't know the distance to the piece, so we attempt to follow this line as closely as possible. This is the most
 * robust way we found to track a piece, as it doesn't rely on the neural network always detecting the piece.</li>
 * </ol>
 */
public class PieceVision extends SubsystemBase {
    private final Alert disconnectedAlert = new Alert("Piece vision camera disconnected!", Alert.AlertType.kError);
    private final PieceVisionIO io;
    private final PieceVisionIOInputsAutoLogged inputs = new PieceVisionIOInputsAutoLogged();
    private boolean wasDisabled = false;

    /**
     * The timeout for tracking a piece. If we don't see a piece for this amount of time, we assume we've lost the
     * piece. This is in seconds.
     */
    private static final double PIECE_TRACKING_TIMEOUT = 0.5;
    /**
     * The angle threshold for tracking a piece. If the piece is within this many degrees of our last observation, we
     * update our locked piece with the new observation. This is in degrees. This also dictates our maximum rotational
     * speed.
     */
    private static final double PIECE_TRACKING_ANGLE_THRESHOLD = 25;

    public static record TargetPath(Translation2d origin, Rotation2d direction) {
        /**
         * Gets a list of two poses on the target path line for AdvantageScope visualization.
         * @return
         */
        public Pose2d[] getVisualizationPoses() {
            return new Pose2d[] {
                new Pose2d(origin, direction),
                new Pose2d(origin, direction).transformBy(new Transform2d(new Translation2d(5.0, 0), Rotation2d.kZero)),
            };
        }

        public Pose2d getPoseAtDistance(double distance) {
            return new Pose2d(origin, direction)
                .transformBy(new Transform2d(new Translation2d(distance, 0), Rotation2d.kZero));
        }
    }

    /**
     * The piece we're currently locked on to. This is used to track a piece over time.
     */
    private PieceLocation lockedPiece = null;
    /**
     * The time we last updated our locked piece. This is used to determine if we have a new observation.
     */
    private double lastUpdate = 0;

    /**
     * The target path of the robot. When we see a piece, we create a line from the robot's position when we made the
     * observation to the piece location. We then attempt to follow this line as closely as possible.
     */
    private TargetPath targetPath = null;

    /**
     * Gets the target path of the robot. This is the path that tracks the piece we're currently locked on to.
     */
    public Optional<TargetPath> getTargetPath() {
        if(targetPath == null) return Optional.empty();
        return Optional.of(targetPath);
    }

    /**
     * The supplier for the robot's current speed. This is used to correct for our current velocity when tracking a
     * piece.
     */
    private final Supplier<ChassisSpeeds> speedSupplier;
    /**
     * The supplier for the robot's current position. This is used to determine the robot's position when we made the
     * observation.
     */
    private final Supplier<Pose2d> positionSupplier;

    public PieceVision(PieceVisionIO io, Supplier<ChassisSpeeds> speedSupplier, Supplier<Pose2d> positionSupplier) {
        this.io = io;
        this.speedSupplier = speedSupplier;
        this.positionSupplier = positionSupplier;

        io.setEnabled(false);
    }

    private PieceLocation getBestPieceLocation(PieceLocations locations) {
        if(locations.locations() == null || locations.locations().length == 0) { return null; }

        PieceLocation best = locations.locations()[0];
        for(PieceLocation location : locations.locations()) {
            if(location.area() > 0.5) {
                // Discard pieces that are too large.
                continue;
            }

            if(location.area() > best.area()) {
                best = location;
            }
        }

        return best;
    }

    /**
     * Updates the target path of the robot. This is used to track a piece over time.
     */
    private void updateTargetPath(double observationTimestamp) {
        if(lockedPiece == null) { return; }

        // Clamp the value just in case something strange happens.
        double latencySeconds = Math.min(observationTimestamp - Timer.getTimestamp(), PIECE_TRACKING_TIMEOUT);
        Logger.recordOutput("PieceVision/Latency", latencySeconds);

        // Correct for our current robot velocity.
        ChassisSpeeds speeds = speedSupplier.get();

        Transform2d velocityOffset = new Transform2d(
            new Translation2d(-speeds.vxMetersPerSecond * latencySeconds, -speeds.vyMetersPerSecond * latencySeconds),
            Rotation2d.fromRadians(-speeds.omegaRadiansPerSecond * latencySeconds));
        Pose2d position = positionSupplier.get();
        Pose2d latencyCompensatedPosition = position.transformBy(velocityOffset);
        // The camera faces backward, so we need to offset by 180 degrees.
        Rotation2d fieldRelativeAngle = latencyCompensatedPosition.getRotation().plus(lockedPiece.theta())
            .plus(Rotation2d.k180deg);
        TargetPath path = new TargetPath(latencyCompensatedPosition.getTranslation(), fieldRelativeAngle);

        Logger.recordOutput("PieceVision/TargetPath", path.getVisualizationPoses());

        targetPath = path;
    }

    /**
     * Performs step 3 of our tracking algorithm. If the piece is near our last observation, we update our locked piece
     * with the new observation. If there isn't a piece within a certain number of degrees of our last observation,
     * we've likely lost the piece. In this case, we wait a set amount of time to see if the piece comes back into view.
     * If it doesn't, we try to find a new piece to track by clearing our locked piece.
     */
    private void trackExistingPiece() {
        if(inputs.locations == null) { return; }

        // Filter only pieces that are within a certain number of degrees of our last observation.
        PieceLocation[] nearbyPieces = new PieceLocation[inputs.locations.locations().length];
        int nearbyPiecesCount = 0;
        for(PieceLocation location : inputs.locations.locations()) {
            if(Math.abs(location.theta().minus(lockedPiece.theta()).getDegrees()) < PIECE_TRACKING_ANGLE_THRESHOLD) {
                nearbyPieces[nearbyPiecesCount++] = location;
            }
        }

        if(nearbyPiecesCount == 0) {
            // We've likely lost the piece. Wait a set amount of time to see if it comes back into view.
            if(Timer.getTimestamp() - lastUpdate > PIECE_TRACKING_TIMEOUT) {
                lockedPiece = null;
                targetPath = null;
            }
        } else {
            // There are pieces near our last observation. Pick the closest to the center of the camera.
            PieceLocation closest = nearbyPieces[0];
            for(int i = 1; i < nearbyPiecesCount; i++) {
                if(Math.abs(nearbyPieces[i].theta().getDegrees()) < Math.abs(closest.theta().getDegrees())) {
                    closest = nearbyPieces[i];
                }
            }

            lockedPiece = closest;
            updateTargetPath(inputs.locations.timestampSeconds());
            lastUpdate = Timer.getTimestamp();
        }
    }

    @Override
    public void periodic() {
        boolean isDisabled = DriverStation.isDisabled();
        if(isDisabled != wasDisabled) {
            io.setEnabled(!isDisabled);
            wasDisabled = isDisabled;

            if(isDisabled) {
                lockedPiece = null;
                targetPath = null;
            }
        }

        io.updateInputs(inputs);

        disconnectedAlert.set(!inputs.connected);

        if(inputs.locations != null) {
            if(lockedPiece == null) {
                // Step 2: Pick the best piece to track.
                lockedPiece = getBestPieceLocation(inputs.locations);
                updateTargetPath(inputs.locations.timestampSeconds());
            } else {
                // Step 3: We're currently locked on to a piece.
                trackExistingPiece();
            }
        }

        LoggedTracer.record("PieceVision");
    }
}
