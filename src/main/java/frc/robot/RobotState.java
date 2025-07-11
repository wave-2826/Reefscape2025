package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.FieldConstants.ReefFace;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.pieceVision.PieceVision;
import frc.robot.subsystems.pieceVision.PieceVision.CoralPosition;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.Vision.IndividualTagEstimate;
import frc.robot.util.DriverStationInterface;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.poseEstimator.OdometrySwerveDrivePoseEstimator;

/**
 * A singleton class that holds the state of the robot. This holds state that doesn't directly control mechanisms and
 * should be accessible across all subsystems. This probably isn't the best way to handle state management, but it makes
 * it easy to quickly prototype and implement new systems, so I'm content with the tradeoff against losing some
 * separation of concerns.
 */
public class RobotState {
    private static final LoggedTunableNumber minSingleTagBlendDistance = new LoggedTunableNumber(
        "Vision/MinSingleTagBlendDistance", Units.inchesToMeters(24.0));
    private static final LoggedTunableNumber maxSingleTagBlendDistance = new LoggedTunableNumber(
        "Vision/MaxSingleTagBlendDistance", Units.inchesToMeters(36.0));

    private static RobotState instance = new RobotState();

    public static RobotState getInstance() {
        return instance;
    }

    private RobotState() {
        // Private constructor to enforce singleton
        for(int i = 1; i <= VisionConstants.aprilTagLayout.getTags().size(); i++) {
            individualTagPoses.put(i, new IndividualTagEstimate(Pose2d.kZero, Double.POSITIVE_INFINITY, -1.0));
        }
    }

    public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.moduleTranslations);
    private Rotation2d rawGyroRotation = Rotation2d.kZero;

    // For delta tracking
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };

    private OdometrySwerveDrivePoseEstimator poseEstimator = new OdometrySwerveDrivePoseEstimator(kinematics,
        rawGyroRotation, lastModulePositions, Pose2d.kZero);

    public Consumer<Pose2d> resetSimulationPoseCallback = (pose) -> {
    };

    /**
     * The tracked positions of coral game pieces.
     */
    public ArrayList<CoralPosition> coralPositions = new ArrayList<>();
    /**
     * If the piece vision camera is disconnected.
     */
    public boolean pieceVisionDisconnected = false;

    @AutoLogOutput(key = "Odometry/RobotVelocity")
    private ChassisSpeeds robotVelocity = new ChassisSpeeds();

    private double elevatorHeightPercent = 0.0;

    public void updateElevatorHeightPercent(double percent) {
        elevatorHeightPercent = percent;
    }

    public double getElevatorHeightPercent() {
        return elevatorHeightPercent;
    }

    private boolean reefLineupSafe = false;

    public void setReefLineupSafe(boolean safe) {
        reefLineupSafe = safe;
    }

    /**
     * Gets whether it's currently safe to line up closely with the reef. This will not be true if the arm is moving
     * upward and will collide.
     * @return
     */
    public boolean isReefLineupSafe() {
        return reefLineupSafe;
    }

    /**
     * Gets the pose at the specified timestamp. This includes vision compensation, so it's a real estimated field pose.
     */
    public Optional<Pose2d> getPoseAtTimestamp(double timestamp) {
        return poseEstimator.sampleAt(timestamp);
    }

    /**
     * Gets the _odometry_ pose at the specified timestamp. This does not include vision compensation, so it will drift
     * over time. This should only be used for delta tracking, not for actual field-relative position.
     * @param inSeconds
     * @return
     */
    public Optional<Pose2d> getOdometryPoseAtTimestamp(double timestamp) {
        return poseEstimator.m_odometryPoseBuffer.getSample(timestamp);
    }

    /**
     * Gets the current _odometry_ pose. This does not include vision compensation, so it will drift over time. This
     * should only be used for delta tracking, not for actual field-relative position.
     * @param inSeconds
     * @return
     */
    public Pose2d getOdometryPose() {
        return poseEstimator.m_odometry.getPoseMeters();
    }

    /**
     * Gets the odometry-based movement from the specified timestamp. This provides a relative transform, and should
     * only be used for timestamps less than 1.5 seconds in the past. The returned pose transforms our pose at the
     * timestamp to our current one.
     * @param timestamp
     * @return
     */
    public Optional<Transform2d> getOdometryMovementSince(double timestamp) {
        var pose = getOdometryPoseAtTimestamp(timestamp);
        return pose.map(p -> new Transform2d(p, getOdometryPose()));
    }

    /** Gets a rough pose prediction in the specified number of seconds based on our velocity. */
    public Pose2d getLookaheadPose(double inSeconds) {
        return getPose().exp(robotVelocity.toTwist2d(inSeconds));
    }

    /**
     * A list of each of the individual tags we currently see. When multiple cameras see the same tag, we trust the
     * camera with the lowest ambiguity.
     * <p>
     * TODO: We should test if it's more reliable to average the robotToTag transforms from all cameras in these cases.
     */
    private final HashMap<Integer, IndividualTagEstimate> individualTagPoses = new HashMap<>();

    /** Gets a pose estimate of the robot based on a specific tag if one is available. */
    public Optional<Pose2d> getIndividualTagRobotPose(int tagId) {
        if(!individualTagPoses.containsKey(tagId)) { return Optional.empty(); }
        var individualTagData = individualTagPoses.get(tagId);

        // If stale, don't use the tag estimate
        if(Timer.getTimestamp() - individualTagData.timestamp() >= Vision.perTagPersistenceTime.get()) {
            return Optional.empty();
        }

        // Latency compensate
        var movement = getOdometryMovementSince(individualTagData.timestamp());
        return movement.map(m -> individualTagData.robotPose().plus(m));
    }

    public static record ReefTagPoseEstimate(Pose2d pose, double interpolation) {
    };

    /**
     * Get an estimated pose using individual tag data given a final pose. Used for reef lineup.
     */
    public ReefTagPoseEstimate getIndividualReefTagPose(ReefFace face, Pose2d finalPose) {
        var individualTagPose = getIndividualTagRobotPose(face.getAprilTagID());
        Pose2d currentPose = getPose();
        if(individualTagPose.isEmpty()) return new ReefTagPoseEstimate(getPose(), 0);

        // Use distance from estimated pose to final pose to interpolate between the two
        final double t = MathUtil.clamp(
            (currentPose.getTranslation().getDistance(finalPose.getTranslation()) - minSingleTagBlendDistance.get())
                / (maxSingleTagBlendDistance.get() - minSingleTagBlendDistance.get()),
            0.0, 1.0);
        return new ReefTagPoseEstimate(currentPose.interpolate(individualTagPose.get(), 1 - t), 1 - t);
    }

    /**
     * Records the observation of an individual tag.
     * @param timestamp
     * @param tagId
     */
    public void addIndividualTagObservation(Pose2d robotPose, double timestamp, double ambiguity, int tagId) {
        // Skip if current data for the tag is newer or it was captured at the same time with lower ambiguity
        if(individualTagPoses.containsKey(tagId)) {
            var estimate = individualTagPoses.get(tagId);
            if(estimate.timestamp() > timestamp) { return; }
            if(timestamp - estimate.timestamp() < 0.05 && estimate.ambiguity() <= ambiguity) { return; }
        }

        var movement = getOdometryMovementSince(timestamp);
        if(movement.isEmpty()) { return; }

        Rotation2d robotRotation = robotPose.transformBy(movement.get().inverse()).getRotation();

        // Use the gyro angle at the capture time for the pose's rotation
        robotPose = new Pose2d(robotPose.getTranslation(), robotRotation);

        // Add transform to current odometry based pose for latency correction
        individualTagPoses.put(tagId, new IndividualTagEstimate(robotPose, ambiguity, timestamp));
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Gets if the robot is currently on the right side of the field.
     * @return
     */
    @AutoLogOutput(key = "Odometry/IsOnRightSide")
    public boolean isOnRightSide() {
        var bluePose = AutoBuilder.shouldFlip() ? FlippingUtil.flipFieldPose(RobotState.getInstance().getPose())
            : RobotState.getInstance().getPose();
        return bluePose.getY() < FieldConstants.fieldWidth / 2.;
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /**
     * Resets the current odometry pose. Called at the start of autonomous so the robot knows what the routine's
     * starting pose is.
     */
    public void setPose(Pose2d pose, SwerveModulePosition[] modulePositions) {
        resetSimulationPoseCallback.accept(pose);
        poseEstimator.resetPosition(rawGyroRotation, modulePositions, pose);
    }

    /** Adds a new timestamped vision measurement. */
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs) {
        // Logger.recordOutput("Vision/StdDevs1", visionMeasurementStdDevs.get(0, 0));
        // Logger.recordOutput("Vision/StdDevs2", visionMeasurementStdDevs.get(1, 0));
        // Logger.recordOutput("Vision/StdDevs3", visionMeasurementStdDevs.get(2, 0));
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    public void applyOdometryUpdate(double timestamp, SwerveModulePosition[] modulePositions,
        Optional<Rotation2d> gyroRotation) {
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
        for(int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
            moduleDeltas[moduleIndex] = new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
            lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
        }

        // Update gyro angle
        if(gyroRotation.isPresent()) {
            // Use the real gyro angle
            rawGyroRotation = gyroRotation.get();
        } else {
            // Use the angle delta from the kinematics and module deltas
            Twist2d twist = kinematics.toTwist2d(moduleDeltas);
            rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        }

        // Apply update
        poseEstimator.updateWithTime(timestamp, rawGyroRotation, modulePositions);
    }

    public void addDriveSpeeds(ChassisSpeeds speeds) {
        robotVelocity = speeds;
    }

    public void addCoralPosition(Translation2d pieceLocation, double timestamp) {
        CoralPosition coralPosition = new CoralPosition(pieceLocation, timestamp);

        coralPositions = coralPositions.stream()
            .filter((p) -> p.translation().getDistance(pieceLocation) > PieceVision.coralOverlap.get())
            .collect(Collectors.toCollection(ArrayList::new));
        coralPositions.add(coralPosition);
    }

    public void resetCoralPositions() {
        coralPositions.clear();
    }

    public void removeOldCoralPositions() {
        coralPositions = coralPositions.stream()
            .filter((x) -> Timer.getTimestamp() - x.timestamp() < PieceVision.coralPersistenceTime.get())
            .collect(Collectors.toCollection(ArrayList::new));
    }

    public ChassisSpeeds getRobotVelocity() {
        return robotVelocity;
    }

    /**
     * Gets the robot's linear velocity in meters per second.
     * @return
     */
    public double getRobotLinearVelocity() {
        return Math.sqrt(Math.pow(robotVelocity.vxMetersPerSecond, 2) + Math.pow(robotVelocity.vyMetersPerSecond, 2));
    }

    public ChassisSpeeds getFieldVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getRotation());
    }

    public Stream<Translation2d> getCoralTranslations() {
        return coralPositions.stream().map(CoralPosition::translation);
    }

    public static Pose3d coralTranslationToVisualizerPose(Translation2d translation) {
        return new Pose3d(new Translation3d(translation.getX(), translation.getY(), FieldConstants.coralDiameter / 2.0),
            new Rotation3d(new Rotation2d(Timer.getTimestamp() * 5.0)));
    }

    /**
     * Updates and logs various periodic state information. This is called every loop iteration.
     */
    public void update() {
        // Update the driver station interface
        DriverStationInterface.getInstance().update(getPose());

        Logger.recordOutput("PieceVision/CoralPoses",
            getCoralTranslations().map(RobotState::coralTranslationToVisualizerPose).toArray(Pose3d[]::new));
    }
}
