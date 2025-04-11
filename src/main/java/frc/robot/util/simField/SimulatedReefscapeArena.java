package frc.robot.util.simField;

import java.util.Arrays;
import java.util.List;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape.ReefscapeFieldObstacleMap;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;
import org.ironmaple.utils.FieldMirroringUtils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A custom reefscape arena simulation that works better for our needs (like allowing our scored coral to go on
 * branches). I would prefer to not need to copy most of the existing class, but a few important parts are private.
 */
public class SimulatedReefscapeArena extends SimulatedArena {
    public final CustomReefSimulation reefSimulation;

    public SimulatedReefscapeArena() {
        super(new ReefscapeFieldObstacleMap());
        reefSimulation = new CustomReefSimulation(this);
        super.addCustomSimulation(reefSimulation);
    }

    @Override
    public void placeGamePiecesOnField() {
        Translation2d[] bluePositions = new Translation2d[] {
            new Translation2d(1.219, 5.855), new Translation2d(1.219, 4.026), new Translation2d(1.219, 2.197),
        };
        for(Translation2d position : bluePositions) super.addGamePiece(new ReefscapeCoralAlgaeStack(position));

        Translation2d[] redPositions = Arrays.stream(bluePositions)
            .map(bluePosition -> new Translation2d(FieldMirroringUtils.FIELD_WIDTH - bluePosition.getX(),
                bluePosition.getY()))
            .toArray(Translation2d[]::new);
        for(Translation2d position : redPositions) super.addGamePiece(new ReefscapeCoralAlgaeStack(position));
    }

    @Override
    public synchronized List<Pose3d> getGamePiecesByType(String type) {
        List<Pose3d> poses = super.getGamePiecesByType(type);

        // add algae and coral stack
        if(type.equals("Algae")) poses.addAll(ReefscapeCoralAlgaeStack.getStackedAlgaePoses());
        else if(type.equals("Coral")) {
            poses.addAll(ReefscapeCoralAlgaeStack.getStackedCoralPoses());
            reefSimulation.addCoralsOnReefForDisplay(poses);
        }

        return poses;
    }

    @Override
    public synchronized void clearGamePieces() {
        super.clearGamePieces();
        reefSimulation.clearReef();
    }
}
