package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    /**
     * Enable drawing a wireframe visualization of the field to the camera streams in simulation mode. This is extremely
     * resource-intensive!
     */
    public static boolean enableWireframeDrawing = false;

    /**
     * If we should enable vision simulation. Turning off vision sim can dramatically improve loop times, but it's
     * obviously less representative of real robot odometry.
     */
    public static boolean enableVisionSimulation = true;

    /**
     * The set of AprilTags used. This needs to be configurable because there are two different sets of AprilTags in
     * 2025... See team update 12: https://firstfrc.blob.core.windows.net/frc2025/Manual/TeamUpdates/TeamUpdate12.pdf
     */
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout
        .loadField(AprilTagFields.k2025ReefscapeWelded);

    // Camera names; must match names configured in PhotonVision
    public static String camera0Name = "2826_OV9281_Abe"; // Front left
    public static String camera1Name = "2826_OV9281_Ben"; // Front right
    public static String camera2Name = "2826_OV9281_Cal"; // Back left
    public static String camera3Name = "2826_OV9281_Dan"; // Back right

    private static Transform3d reflectCameraPosition(Transform3d pos) {
        Translation3d translation = pos.getTranslation();
        translation = new Translation3d(translation.getX(), -translation.getY(), translation.getZ());
        Rotation3d rotation = pos.getRotation();
        rotation = new Rotation3d(rotation.getX(), rotation.getY(), -rotation.getZ());
        return new Transform3d(translation, rotation);
    }

    // Robot to camera transforms
    // Front left camera
    public static Transform3d robotToCamera0 = new Transform3d(
        new Translation3d(0.27411734834696505, 0.2519450092311559, 0.37164545076967626),
        new Rotation3d(0.003627879341338477, 0.48105665399639214, -0.5622702127309405));
    // Front right camera
    public static Transform3d robotToCamera1 = new Transform3d(
        new Translation3d(0.26895335537009596, -0.2128584446109858, 0.3670752799208795),
        new Rotation3d(0.01621000946342666, 0.47716456943182917, 0.5355458463649192));
    // Back left camera
    public static Transform3d robotToCamera2 = new Transform3d(new Translation3d(Units.inchesToMeters(4.006715),
        Units.inchesToMeters(7.487012), Units.inchesToMeters(25.736304)),
        new Rotation3d(0.0, 0.0, -0.523598 - Math.PI));
    // Back right camera
    public static Transform3d robotToCamera3 = reflectCameraPosition(robotToCamera2);

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.015; // Meters
    public static double angularStdDevBaseline = 1000.; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
        1.0, // Camera 0
        1.0, // Camera 1
        1.5, // Camera 2
        1.5 // Camera 3
    };
}
