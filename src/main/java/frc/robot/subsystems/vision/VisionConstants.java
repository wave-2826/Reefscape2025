package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    // Enable drawing a wireframe visualization of the field to the camera streams in simulation mode.
    // This is extremely resource-intensive!
    public static boolean enableWireframeDrawing = false;

    // If we should enable vision simulation.
    // Turning off vision sim can dramatically improve loop times, but it's obviously less
    // representative of real robot odometry.
    public static boolean enableVisionSimulation = true;

    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

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
    public static Transform3d robotToCamera0 = new Transform3d(new Translation3d(Units.inchesToMeters(11.55791101),
        Units.inchesToMeters(12.07248480), Units.inchesToMeters(13.302)), new Rotation3d(0.0, 0.319624, -0.680146));
    // Front right camera
    public static Transform3d robotToCamera1 = reflectCameraPosition(robotToCamera0);
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
