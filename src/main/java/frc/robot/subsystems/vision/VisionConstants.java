package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {
    /****** Simulation ******/
    /**
     * If we should enable vision simulation. Turning off vision sim can dramatically improve loop times, but it's
     * obviously far less representative of real robot odometry.
     */
    public static final boolean enableVisionSimulation = false;

    /**
     * Enable drawing a wireframe visualization of the field to the camera streams in simulation mode. This is extremely
     * resource-intensive!
     */
    public static final boolean enableWireframeDrawing = false;

    /**
     * Enable raw streams for simulated cameras. This can increase loop times slightly.
     */
    public static final boolean enableRawStreams = true;
    /************************/

    /**
     * The set of AprilTags used. This needs to be configurable because there are two different sets of AprilTags in
     * 2025... See team update 12: https://firstfrc.blob.core.windows.net/frc2025/Manual/TeamUpdates/TeamUpdate12.pdf
     */
    public static final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout
        .loadField(AprilTagFields.k2025ReefscapeWelded);

    // Camera names; must match names configured in PhotonVision
    public static final String camera0Name = "2826_OV9281_Ben"; // Front left
    public static final String camera1Name = "2826_OV9281_Abe"; // Front right
    public static final String camera2Name = "2826_OV9281_Cal"; // Back left
    public static final String camera3Name = "2826_OV9281_Dan"; // Back right

    // Robot to camera transforms
    // Front left camera
    public static final Transform3d robotToCamera0 = new Transform3d(
        new Translation3d(0.2641188936435686, 0.27179662045243047, 0.3268217346775703),
        new Rotation3d(0.009216829869306487, 0.25623936411599274, -0.5882221880563707));
    // Front right camera
    public static final Transform3d robotToCamera1 = new Transform3d(
        new Translation3d(0.2636302545668782, -0.27179662045243047, 0.31824257640474607),
        new Rotation3d(0.021219301471488785, 0.22761117681482465, 0.5872378479679213));
    // Back left camera
    public static final Transform3d robotToCamera2 = new Transform3d(
        new Translation3d(0.17616398505194292, 0.23123207781818467, 0.6676432845715006),
        new Rotation3d(0.0027423172023952245, 0.026004347701025454, 2.64243891600842));
    // Back right camera
    public static final Transform3d robotToCamera3 = new Transform3d(
        new Translation3d(0.10200963510691768, -0.20143263871228373, 0.6624475448167352),
        new Rotation3d(0.0012974505131851419, -0.014898991823721702, -2.4933118982608544));

    // Basic filtering thresholds
    public static final double maxAmbiguity = 0.3;
    public static final double maxZError = 0.75;
    /** The maximum error in an estimate's rotation in degrees. */
    public static final double maxRotationError = 20;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static final double linearStdDevBaseline = 0.05; // Meters
    public static final double angularStdDevBaseline = 10000.; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
        1.0, // Camera 0
        1.0, // Camera 1
        1.5, // Camera 2
        1.5 // Camera 3
    };
}
