package frc.robot.subsystems.vision;

/** IO implementation for real Limelight hardware using a neural network classifier pipeline. */
public class VisionIOLimelightNeural implements VisionIO {
    private String hostname;

    public VisionIOLimelightNeural(String hostname) {
        this.hostname = hostname;
    }
    
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        
    }
}
