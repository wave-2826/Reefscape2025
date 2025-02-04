package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
    // Enable drawing a wireframe visualization of the field to the camera streams in simulation mode.
    // This is extremely resource-intensive!
    public static boolean enableWireframeDrawing = false;

    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Camera names; must match names configured in PhotonVision
    public static String camera0Name = "frontLeftCamera";
    public static String camera1Name = "frontRightCamera";
    public static String camera2Name = "backLeftCamera";
    public static String camera3Name = "backRightCamera";

    // Robot to camera transforms
    public static Transform3d robotToCamera0 = new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
    public static Transform3d robotToCamera1 = new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));
    public static Transform3d robotToCamera2 = new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
    public static Transform3d robotToCamera3 = new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
        1.0, // Camera 0
        1.0, // Camera 1
        1.0, // Camera 2
        1.0 // Camera 3
    };
}
