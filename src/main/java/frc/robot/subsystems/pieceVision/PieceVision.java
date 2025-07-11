package frc.robot.subsystems.pieceVision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.pieceVision.PieceVisionIO.PieceLocation;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;

/**
 * Handles autonomous tracking of game pieces. This is separated from the main vision subsystem because it doesn't need
 * to interact and it makes the code cleaner.
 * 
 * If we have a new observation, continue tracking a piece. Our tracking system needs to be pretty sophisticated to
 * handle the robot moving, piece moving, pieces flickering in and out, and multiple pieces simultaneously. This leaves
 * a ton of possible edge cases to handle, so we need to be very careful.
 */
public class PieceVision extends SubsystemBase {
    public static final LoggedTunableNumber coralOverlap = new LoggedTunableNumber("PieceVision/CoralOverlap", .5);
    public static final LoggedTunableNumber coralPersistenceTime = new LoggedTunableNumber(
        "PieceVision/CoralPersistenceTime", 1.0);

    public record CoralPosition(Translation2d translation, double timestamp) {
    }

    private final Alert disconnectedAlert = new Alert("Piece vision camera disconnected!", Alert.AlertType.kError);

    private final PieceVisionIO io;
    private final PieceVisionIOInputsAutoLogged inputs = new PieceVisionIOInputsAutoLogged();
    private boolean wasDisabled = false;

    public PieceVision(PieceVisionIO io) {
        this.io = io;

        io.setEnabled(false);

        // Preload the records that need to be logged to avoid stalling the code
        Logger.recordOutput("PieceVision/PieceLocations", new PieceLocation[] {});
    }

    @Override
    public void periodic() {
        boolean isDisabled = DriverStation.isDisabled();
        if(isDisabled != wasDisabled) {
            io.setEnabled(!isDisabled);
            wasDisabled = isDisabled;
        }

        io.updateInputs(inputs);
        Logger.processInputs("PieceVision", inputs);

        disconnectedAlert.set(!inputs.connected);

        var robotState = RobotState.getInstance();
        robotState.pieceVisionDisconnected = !inputs.connected;

        if(inputs.locations != null) {
            var locations = inputs.locations;

            var robotPose = robotState.getPose();
            var robotPose3d = new Pose3d(robotPose.getTranslation().getX(), robotPose.getTranslation().getY(), 0.0,
                new Rotation3d(0.0, 0.0, robotPose.getRotation().getRadians()));
            Pose3d cameraOnRobot = robotPose3d.plus(PieceVisionConstants.robotToCamera);

            for(int i = 0; i < locations.length; i++) {
                // Project the pitch and yaw of the observation onto the field plane (plus half a coral height) to estimate the location of the piece.
                var location = locations[i];
                var pose = cameraOnRobot.transformBy(new Transform3d(new Translation3d(),
                    new Rotation3d(0, -location.pitch().getRadians(), location.theta().getRadians())));

                // Positive pitch is downward
                if(pose.getRotation().getY() < Units.degreesToRadians(3)) {
                    // The piece angle is close to the ground plane angle; this observation doesn't make sense and will give a super far away piece
                    continue;
                }

                // Solve for the X and Y position of the piece
                // We projecting to a Z coordinate of half the coral height. This is only correct for horizontal coral,
                // but it's probably true almost all of the time.
                var distanceToFloorLocation = (PieceVisionConstants.robotToCamera.getZ()
                    - FieldConstants.coralDiameter / 2) / Math.sin(pose.getRotation().getY());
                Translation2d pieceLocation = pose
                    .transformBy(
                        new Transform3d(new Translation3d(distanceToFloorLocation, 0.0, 0.0), Rotation3d.kZero))
                    .toPose2d().getTranslation();

                robotState.addCoralPosition(pieceLocation, inputs.timestamp);
            }
            robotState.removeOldCoralPositions();
        }

        LoggedTracer.record("PieceVision");
    }
}
