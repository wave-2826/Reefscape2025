package frc.robot;

import java.util.Optional;
import java.util.function.Consumer;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.DriverStationInterface;

/**
 * A singleton class that holds the state of the robot. This holds state that doesn't directly control mechanisms and
 * should be accessible across all subsystems. This probably isn't the best way to handle state management, but it makes
 * it easy to quickly prototype and implement new systems, so I'm content with the tradeoff against losing some
 * separation of concerns.
 */
public class RobotState {
    private static RobotState instance = null;

    public static RobotState getInstance() {
        if(instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    private RobotState() {
        // Private constructor to enforce singleton
    }

    public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.moduleTranslations);
    private Rotation2d rawGyroRotation = Rotation2d.kZero;

    // For delta tracking
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };

    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation,
        lastModulePositions, Pose2d.kZero);

    public Consumer<Pose2d> resetSimulationPoseCallback = (pose) -> {
    };

    private static final double poseBufferSizeSeconds = 1.0;
    /**
     * An interpolatable buffer of the robot pose. This is used to reference past odometry measurements when referring
     * to target observations.
     */
    private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer
        .createBuffer(poseBufferSizeSeconds);

    @AutoLogOutput(key = "Odometry/RobotVelocity")
    private ChassisSpeeds robotVelocity = new ChassisSpeeds();

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
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
        poseBuffer.clear();
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

    public ChassisSpeeds getFieldVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getRotation());
    }

    /**
     * Updates various periodic state information. This is called every loop iteration.
     */
    public void update() {
        // Update the driver station interface
        DriverStationInterface.getInstance().updateRobotPose(getPose());
    }
}
