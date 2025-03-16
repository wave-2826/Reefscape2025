package frc.robot.subsystems.pieceVision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.pieceVision.PieceVisionIO.PieceLocation;
import frc.robot.subsystems.pieceVision.PieceVisionIO.PieceLocations;
import frc.robot.util.LoggedTracer;

/**
 * Handles autonomous tracking of game pieces. This is separated from the main vision subsystem because it doesn't need
 * to interact and it makes the code cleaner.
 */
public class PieceVision extends SubsystemBase {
    private final Alert disconnectedAlert = new Alert("Piece vision camera disconnected!", Alert.AlertType.kError);
    private final PieceVisionIO io;
    private final PieceVisionIOInputsAutoLogged inputs = new PieceVisionIOInputsAutoLogged();
    private boolean wasDisabled = false;

    private static record TargetPath(Translation2d origin, Rotation2d direction) {
    }

    /**
     * The piece we're currently locked on to. This is used to track a piece over time.
     */
    private PieceLocation lockedPiece = null;
    /**
     * The target path of the robot. When we see a piece, we create a line from the robot's position when we made the
     * observation to the piece location. We then attempt to follow this line as closely as possible.
     */
    private TargetPath targetPath = null;

    public PieceVision(PieceVisionIO io) {
        this.io = io;

        io.setEnabled(false);
    }

    public Command followTargetLine(Drive drive) {
        return Commands.run(() -> {
            // drive.runVelocity(ChassisSpeeds.)
        }, drive);
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

    @Override
    public void periodic() {
        boolean isDisabled = DriverStation.isDisabled();
        if(isDisabled != wasDisabled) {
            io.setEnabled(!isDisabled);
            wasDisabled = isDisabled;
        }

        io.updateInputs(inputs);

        disconnectedAlert.set(!inputs.connected);

        // If we have a new observation, continue tracking a piece.
        // Our tracking system needs to be pretty sophisticated to handle the robot moving,
        // piece moving, pieces flickering in and out, and multiple pieces simultaneously.
        // This leaves a ton of possible edge cases to handle, so we need to be very careful.

        // Our algorithm works like this:
        // 1. If we don't have a new observation, do nothing.
        // 2. If we're not currently "locked on" to a piece, pick the best piece to track.
        //    We currently determine this by the piece with the largest area.
        //    However, we discard pieces that are too large (>50% of the screen). This may
        //    not be necessary, but it's an additional safeguard against the neural network
        //    detecting something that isn't a piece.
        // 3. If we are currently locked on to a piece, perform the following:
        //    a. If there isn't a piece within a certain number of degrees of our last observation,
        //       we've likely lost the piece.
        //       In this case, we wait a set amount of time to see if the piece comes back into view.
        //       If it doesn't, we try to find a new piece to track by clearing our locked piece.
        //    b. If the piece is near our last observation, we update our locked piece with the new observation.
        // 4. If we have a locked piece, we create a line from where our robot was when the
        //    observation occured to the piece. We don't know the distance to the piece, so
        //    we attempt to follow this line as closely as possible. This is the most robust
        //    way we found to track a piece, as it doesn't rely on the neural network always
        //    detecting the piece.

        // TODO: Implement the above algorithm.
        if(inputs.locations != null) {
            if(lockedPiece == null) {
                // Step 2: Pick the best piece to track.
                lockedPiece = getBestPieceLocation(inputs.locations);
                // TODO: Update target path
            }
        }

        LoggedTracer.record("PieceVision");
    }
}
