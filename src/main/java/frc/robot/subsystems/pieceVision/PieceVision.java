package frc.robot.subsystems.pieceVision;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.util.LoggedTracer;

/**
 * Handles autonomous tracking of game pieces. This is separated from the main vision subsystem because it doesn't need
 * to interact and it makes the code cleaner.
 */
public class PieceVision extends SubsystemBase {
    private final Alert disconnectedAlert = new Alert("Piece vision camera disconnected!", Alert.AlertType.kError);
    private final PieceVisionIO io;
    private final PieceVisionIOInputsAutoLogged inputs = new PieceVisionIOInputsAutoLogged();

    public PieceVision(PieceVisionIO io) {
        this.io = io;

        io.setEnabled(false);
        RobotModeTriggers.disabled().onFalse(Commands.runOnce(() -> io.setEnabled(false)))
            .onTrue(Commands.runOnce(() -> io.setEnabled(true)));
    }

    @Override
    public void periodic() {
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

        LoggedTracer.record("PieceVision");
    }
}
