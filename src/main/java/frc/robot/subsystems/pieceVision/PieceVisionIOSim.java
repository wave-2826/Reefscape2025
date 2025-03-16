package frc.robot.subsystems.pieceVision;

import edu.wpi.first.wpilibj.Timer;

public class PieceVisionIOSim implements PieceVisionIO {
    // TODO
    private static double FRAMERATE = 25.;

    public PieceVisionIOSim() {
    }

    @Override
    public void updateInputs(PieceVisionIOInputs inputs) {
        inputs.connected = true;
        inputs.cpuTemp = 25;
        inputs.framerate = FRAMERATE;
        inputs.locations = new PieceLocations(Timer.getTimestamp(), new PieceLocation[0]);
    }
}
