package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionIO.SingleApriltagResult;
import frc.robot.util.LoggedTracer;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private final VisionConsumer consumer;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;

    private static record RobotToTag(Transform3d robotToTag, double ambiguity, double timestamp) {
    }

    /**
     * A map from the robot position to each of the individual tags we currently see. When multiple cameras see the same
     * tag, we trust the camera with the lowest ambiguity.
     * <p>
     * TODO: We should test if it's more reliable to average the robotToTag transforms from all cameras in these cases.
     */
    private final HashMap<Integer, RobotToTag> robotToIndividualTags = new HashMap<>();

    public Vision(VisionConsumer consumer, VisionIO... io) {
        this.consumer = consumer;
        this.io = io;

        // Initialize inputs
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for(int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[io.length];
        for(int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] = new Alert("Vision camera " + Integer.toString(i) + " is disconnected.",
                AlertType.kWarning);
        }
    }

    /**
     * Get the transform from the robot to the tag with the given ID. Returns null if the tag is not seen.
     * @param id
     * @return
     */
    public Transform3d getRobotToTag(int id) {
        if(robotToIndividualTags.containsKey(id)) { return robotToIndividualTags.get(id).robotToTag; }
        return null;
    }

    public int getCameraCount() {
        return io.length;
    }

    @Override
    public void periodic() {
        if(Constants.currentMode == Constants.Mode.SIM && !VisionConstants.enableVisionSimulation) { return; }

        for(int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
        }

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        HashMap<Integer, RobotToTag> currentIndividualTags = new HashMap<>();

        // Loop over cameras
        for(int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            if(inputs[cameraIndex].individualTags != null) {
                for(SingleApriltagResult result : inputs[cameraIndex].individualTags) {
                    int id = result.fiducialId();

                    var tagPose = aprilTagLayout.getTagPose(id);
                    if(tagPose.isPresent()) tagPoses.add(tagPose.get());

                    // Add to robotToIndividualTags map if it isn't obviously wrong
                    Transform3d transform = result.robotToTarget();
                    boolean rejectPose = result.ambiguity() > maxAmbiguity // Cannot be high ambiguity
                        || Math.abs(transform.getZ()) > maxZError // Must have realistic Z coordinate
                        // Must be within the field boundaries
                        || transform.getX() < 0.0 || transform.getX() > aprilTagLayout.getFieldLength()
                        || transform.getY() < 0.0 || transform.getY() > aprilTagLayout.getFieldWidth();

                    if(rejectPose) continue;

                    if(!currentIndividualTags.containsKey(id)
                        || result.ambiguity() < currentIndividualTags.get(id).ambiguity()) {
                        Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotToTag" + id,
                            transform);
                        currentIndividualTags.put(id,
                            new RobotToTag(transform, result.ambiguity(), Timer.getTimestamp()));
                    }
                }
            }

            // Loop over pose observations
            for(var observation : inputs[cameraIndex].poseObservations) {
                // Check whether to reject pose
                boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
                    || (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity) // Cannot be high
                    // Ambiguity
                    || Math.abs(observation.pose().getZ()) > maxZError // Must have realistic Z coordinate

                    // Must be within the field boundaries
                    || observation.pose().getX() < 0.0 || observation.pose().getX() > aprilTagLayout.getFieldLength()
                    || observation.pose().getY() < 0.0 || observation.pose().getY() > aprilTagLayout.getFieldWidth();

                // Add pose to log
                robotPoses.add(observation.pose());
                if(rejectPose) robotPosesRejected.add(observation.pose());
                else robotPosesAccepted.add(observation.pose());

                // Skip if rejected
                if(rejectPose) continue;

                // Calculate standard deviations
                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = linearStdDevBaseline * stdDevFactor;
                double angularStdDev = angularStdDevBaseline * stdDevFactor;
                if(cameraIndex < cameraStdDevFactors.length) {
                    linearStdDev *= cameraStdDevFactors[cameraIndex];
                    angularStdDev *= cameraStdDevFactors[cameraIndex];
                }

                // Send vision observation
                consumer.accept(observation.pose().toPose2d(), observation.timestamp(),
                    VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
            }

            // Log camera datadata
            Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
                tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
                robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
                robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
                robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));

            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Update robotToIndividualTags
        // If any single tag hasn't been updated for a quarter of a second, clear it.
        double currentTime = Timer.getTimestamp();
        robotToIndividualTags.entrySet().removeIf(entry -> currentTime - entry.getValue().timestamp > 0.25);
        robotToIndividualTags.putAll(currentIndividualTags);

        // Log summary data
        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        Logger.recordOutput("Vision/Summary/RobotPosesAccepted",
            allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput("Vision/Summary/RobotPosesRejected",
            allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));

        LoggedTracer.record("Vision");
    }

    public Transform3d[] getBestTagTransforms() {
        Transform3d[] results = new Transform3d[io.length];
        for(int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            results[cameraIndex] = inputs[cameraIndex].bestTagTransform;
        }
        return results;
    }

    public String[] getCameraNames() {
        String[] names = new String[io.length];
        for(int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            names[cameraIndex] = io[cameraIndex].getName();
        }
        return names;
    }

    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
