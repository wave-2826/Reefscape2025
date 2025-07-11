package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.maxSpeedMetersPerSec;
import static frc.robot.subsystems.drive.DriveConstants.maxSteerVelocity;
import static frc.robot.subsystems.drive.DriveConstants.moduleTranslations;
import static frc.robot.subsystems.drive.DriveConstants.pathplannerConfig;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.Constants.Mode;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.LoggedTracer;

import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The drivetrain subsystem. Manages the swerve drive including all modules, the gyro, kinematics, odometry, and system
 * identification.
 */
public class Drive extends SubsystemBase {
    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.",
        AlertType.kError);

    public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.moduleTranslations);

    // See https://pathplanner.dev/pplib-swerve-setpoint-generator.html
    // We use PathPlanner's variation of 254's setpoint generator which takes a prior setpoint,
    // a desired setpoint, and outputs a new setpoint that respects all the kinematic constraints
    // on module rotation and wheel velocity/torque, as well as preventing any forces acting on a
    // module's wheel from exceeding the force of friction.
    private final SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;

    /** If the setpoints have been updated this loop. Used to avoid continuing at a speed when we tell it to stop. */
    private boolean setpointsUpdated = false;
    /** If we're currently controlling the robot with velocity. */
    private boolean velocityControlMode = true;
    /** The latest speed setpoint for velocity control. */
    private ChassisSpeeds latestSpeedSetpoint = new ChassisSpeeds();

    /** The acceleration that needs to be experienced for an "abrupt stop". */
    private final static double ABRUPT_STOP_THRESHOLD_GS = 7.0;
    /** Whether the robot experienced an abrupt stop last loop iteration. Used to avoid multiple callbacks. */
    private boolean previousAbruptStop = false;
    /** A callback that will be called if the robot hits a wall or is otherwise abruptly accelerated. */
    private static Runnable abruptStopCallback = null;

    /** A debouncer that automatically unlocks the wheels after the robot has been disabled for a period of time. */
    private final Debouncer unlockWheelsDebouncer = new Debouncer(2.0, Debouncer.DebounceType.kFalling);
    /** If the wheels are currently in brake mode. */
    private boolean wheelsLocked = true;

    public static void setAbruptStopCommand(Command command) {
        Drive.abruptStopCallback = command::schedule;
    }

    /**
     * Constructs a new Drive subsystem.
     * @param gyroIO
     * @param flModuleIO
     * @param frModuleIO
     * @param blModuleIO
     * @param brModuleIO
     * @param resetSimulationPoseCallBack A callback that will be called when the robot pose is updated, like at the
     *            start of autonomous. This is used to also reset the pose in simulation.
     */
    public Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;

        modules[0] = new Module(flModuleIO, "FrontLeft");
        modules[1] = new Module(frModuleIO, "FrontRight");
        modules[2] = new Module(blModuleIO, "BackLeft");
        modules[3] = new Module(brModuleIO, "BackRight");

        // Start odometry thread
        SparkOdometryThread.getInstance().start();

        // Configure AutoBuilder for PathPlanner
        var robotState = RobotState.getInstance();
        AutoBuilder.configure(robotState::getPose, this::setPose, this::getChassisSpeeds,
            this::runVelocityWithFeedforward,
            Constants.isSim ? DriveConstants.simHolonomicDriveController : DriveConstants.realHolonomicDriveController,
            pathplannerConfig, () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
            // HACK: what is pathplanner doing
            if(!Constants.isSim) {
                boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                if(DriverStation.isEnabled() && isRed && targetPose.getX() < FieldConstants.fieldLength / 2) {
                    System.out.println("Stopping auto; PathPlanner is trying to destroy the robot again");
                    CommandScheduler.getInstance().cancelAll();
                    DriveCommands.driveStraightCommand(this, Units.feetToMeters(2), 1,
                        () -> FlippingUtil.flipFieldRotation(Rotation2d.kZero)).schedule();
                }
                if(DriverStation.isEnabled() && !isRed && targetPose.getX() > FieldConstants.fieldLength / 2) {
                    System.out.println("Stopping auto; PathPlanner is trying to destroy the robot again");
                    CommandScheduler.getInstance().cancelAll();
                    DriveCommands.driveStraightCommand(this, Units.feetToMeters(2), 1,
                        () -> FlippingUtil.flipFieldRotation(Rotation2d.kZero)).schedule();
                }
            }

            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

        // Due to how Java works, the first run of pathfinding commands can have significantly higher delay than subsequent runs.
        // This can be partially alleviated by running a warmup command in the background when code starts.
        // It won't control the robot; it simply runs through a full pathfinding command to warm up the library.
        PathfindingCommand.warmupCommand().schedule();

        setpointGenerator = new SwerveSetpointGenerator(pathplannerConfig, maxSteerVelocity);
        previousSetpoint = new SwerveSetpoint(getChassisSpeeds(), getModuleStates(), DriveFeedforwards.zeros(4));
    }

    /**
     * Runs the drive at the desired robot-relative velocity with the specified feedforwards.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocityWithFeedforward(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
        runVelocity(speeds, feedforwards.accelerationsMPSSq());
    }

    /**
     * Runs the drive at the desired robot-relative velocity. Doesn't account for acceleration.
     * @param speeds
     * @param accelerations
     */
    public void runVelocity(ChassisSpeeds speeds) {
        runVelocity(speeds, new double[4]);
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    @SuppressWarnings("unused")
    public void runVelocity(ChassisSpeeds speeds, double[] accelerationsMps2) {
        Logger.recordOutput("SwerveChassisSpeeds/TargetSpeeds", speeds);

        setpointsUpdated = true;
        velocityControlMode = true;
        latestSpeedSetpoint = speeds;

        // Calculate module setpoints
        // We only use the setpoint generator in teleoperated since Choreo already only generates possible trajectories.
        SwerveModuleState[] setpointStates;
        if(DriveConstants.USE_SETPOINT_GENERATOR && DriverStation.isTeleop()) {
            previousSetpoint = setpointGenerator.generateSetpoint(previousSetpoint, speeds, 0.02);
            setpointStates = previousSetpoint.moduleStates();
            accelerationsMps2 = previousSetpoint.feedforwards().accelerationsMPSSq();
        } else {
            ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
            setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);
        }

        // Log unoptimized setpoints
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);

        // Send setpoints to modules
        for(int i = 0; i < 4; i++) modules[i].runSetpoint(setpointStates[i], accelerationsMps2[i]);

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        velocityControlMode = false;
        for(int i = 0; i < 4; i++) modules[i].runCharacterization(output);
    }

    /** Runs the drive to rotate with the specified drive output for angular system identification. */
    public void runAngularCharacterization(double output) {
        velocityControlMode = false;
        for(int i = 0; i < 4; i++) modules[i].runAngularCharacterization(output);
    }

    /** Runs a particular module in a straight line with the specified drive output. */
    public void runCharacterization(int module, double output) {
        velocityControlMode = false;
        modules[module].runCharacterization(output);
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /** Resets the modules to absolute. */
    public void resetToAbsolute() {
        for(int i = 0; i < 4; i++) modules[i].resetToAbsolute();
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will return to their
     * normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for(int i = 0; i < 4; i++) headings[i] = moduleTranslations[i].getAngle();
        kinematics.resetHeadings(headings);
        stop();
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the modules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(int i = 0; i < 4; i++) states[i] = modules[i].getState();
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the modules.
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for(int i = 0; i < 4; i++) states[i] = modules[i].getPosition();
        return states;
    }

    /**
     * Resets the odometry to the specified pose. Used at the start of autonomous to tell the robot where it is.
     * @return
     */
    public void setPose(Pose2d pose) {
        RobotState.getInstance().setPose(pose, getModulePositions());
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns the measured linear speed of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/MeasuredLinear")
    public double getLinearSpeedMetersPerSec() {
        ChassisSpeeds speeds = getChassisSpeeds();
        return Math.sqrt(
            speeds.vxMetersPerSecond * speeds.vxMetersPerSecond + speeds.vyMetersPerSecond * speeds.vyMetersPerSecond);
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for(int i = 0; i < 4; i++) values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        return values;
    }

    /** Returns the average velocity of the modules in rad/sec. */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for(int i = 0; i < 4; i++) output += modules[i].getFFCharacterizationVelocity() / 4.0;
        return output;
    }

    /** Returns the drive motor current draw of a particular module in amps. */
    public double getSlipMeasurementCurrent(int module) {
        return modules[module].getSlipMeasurementCurrent();
    }

    /** Returns the drive motor position of a particular module in radians. */
    public double getSlipMeasurementPosition(int module) {
        return modules[module].getWheelRadiusCharacterizationPosition();
    }

    /** Temporarily changes the drive motor current limit for slip current measurement. */
    public void setSlipMeasurementCurrentLimit(int amps) {
        for(int i = 0; i < 4; i++) modules[i].setSlipMeasurementCurrentLimit(amps);
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for(var module : modules) module.periodic();
        odometryLock.unlock();

        // Stop moving when disabled
        if(DriverStation.isDisabled()) {
            for(var module : modules) module.stop();
        }

        // Unlock wheels if we've been disabled for a while
        boolean shouldLock = unlockWheelsDebouncer.calculate(DriverStation.isEnabled());
        if(shouldLock != wheelsLocked) {
            wheelsLocked = shouldLock;
            for(var module : modules) module.setBrakeMode(wheelsLocked);
        }

        // Log empty setpoint states when disabled
        if(DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        var robotState = RobotState.getInstance();
        for(int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            for(int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
            }

            robotState.applyOdometryUpdate(sampleTimestamps[i], modulePositions,
                gyroInputs.connected ? Optional.of(gyroInputs.odometryYawPositions[i]) : Optional.empty());
        }
        robotState.addDriveSpeeds(getChassisSpeeds());

        // Call the abrupt stop callback if needed
        if(abruptStopCallback != null) {
            if(gyroInputs.accelerationGs > ABRUPT_STOP_THRESHOLD_GS && !previousAbruptStop) {
                abruptStopCallback.run();
                previousAbruptStop = true;
            } else if(gyroInputs.accelerationGs < 0.5) {
                previousAbruptStop = false;
            }
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

        // Subsystems are updated first, so this adds a bit of latency. Therefore,
        // we only update when our commands already didn't immediately update.
        // If they didn't, we manually run velocity control to avoid situations where the robot
        // continues moving after we tell it to stop.
        if(!setpointsUpdated && velocityControlMode && DriverStation.isEnabled()) {
            runVelocity(latestSpeedSetpoint);
        }
        setpointsUpdated = false;

        LoggedTracer.record("Drive");
    }
}
