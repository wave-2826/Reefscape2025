package frc.robot.commands.drive;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.LEDState;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.LoggedTunableNumber;

public class CloseLineupCommand extends Command {
    private static final BooleanSupplier useSingleTag = DriverStation::isAutonomous;

    private final Drive drive;
    private final Vision vision;
    private final LEDs leds;

    private final int tagToTrack;

    private final Transform2d tagRelativeOffset;
    private final Supplier<Transform2d> fieldRelativeOffset;

    private final HolonomicDriveController driveController;

    private final Optional<BooleanSupplier> finishSequence;
    private final BooleanConsumer lineupFeedback;

    private final static LoggedTunableNumber translationKp = new LoggedTunableNumber(//
        "CloseLineup/translationKp", 7.0);
    private final static LoggedTunableNumber translationKi = new LoggedTunableNumber(//
        "CloseLineup/translationKi", 0.0);
    private final static LoggedTunableNumber translationKd = new LoggedTunableNumber(//
        "CloseLineup/translationKd", 0.75);

    private final static LoggedTunableNumber thetaRotationKp = new LoggedTunableNumber(//
        "CloseLineup/thetaRotationKp", 7.0);
    private final static LoggedTunableNumber thetaRotationKi = new LoggedTunableNumber(//
        "CloseLineup/thetaRotationKi", 0.0);
    private final static LoggedTunableNumber thetaRotationKd = new LoggedTunableNumber(//
        "CloseLineup/thetaRotationKd", 0.3);

    private final static LoggedTunableNumber translationTolerance = new LoggedTunableNumber(//
        "CloseLineup/translationTolerance", 0.65);
    private final static LoggedTunableNumber thetaTolerance = new LoggedTunableNumber(//
        "CloseLineup/thetaTolerance", 2.0);

    private final static LoggedTunableNumber maxVelocity = new LoggedTunableNumber(//
        "CloseLineup/maxThetaVelocity", 360); // deg/s
    private final static LoggedTunableNumber maxAcceleration = new LoggedTunableNumber(//
        "CloseLineup/maxThetaAcceleration", 360 * 3.); // deg/s^2

    private final static LoggedTunableNumber thetaIZone = new LoggedTunableNumber(//
        "CloseLineup/thetaIZone", 1.0);

    private final Debouncer ledDebouncer = new Debouncer(0.3, DebounceType.kFalling);
    private final Debouncer feedbackDebouncer = new Debouncer(0.15, DebounceType.kFalling);

    private boolean inRadiusDeadband = false;
    private boolean inThetaDeadband = false;

    private final static LoggedTunableNumber radiusInnerDeadband = new LoggedTunableNumber(//
        "CloseLineup/Deadband/RadiusInnerDeadband", 0.1); // Meters/sec
    private final static LoggedTunableNumber radiusOuterDeadband = new LoggedTunableNumber(//
        "CloseLineup/Deadband/RadiusOuterDeadband", 0.2); // Meters/sec
    private final static LoggedTunableNumber thetaInnerDeadband = new LoggedTunableNumber(//
        "CloseLineup/Deadband/ThetaInnerDeadband", 0.5); // Degrees
    private final static LoggedTunableNumber thetaOuterDeadband = new LoggedTunableNumber(//
        "CloseLineup/Deadband/ThetaOuterDeadband", 2); // Degrees

    private final Debouncer atSetpointDebouncer = new Debouncer(0.25, DebounceType.kRising);

    /**
     * A command that lines up the robot based on tracking the relative position of a single tag from the vision system
     * with PID control. We found this to be the most reliable mechanism to automatically align because it doesn't
     * depend on all tags the robot sees being in precise locations.
     * <p>
     * Internally, this actually _does_ track a field-relative position to allow us to take advantage of drivetrain
     * odometry to run the PID loop faster. However, the target pose is always relative to the tag being tracked.
     * <p>
     * If the tag being tracked is not seen, this falls back to full field-relative tracking.
     * @param drive
     * @param vision
     * @param tagToTrack
     * @param tagRelativeOffset The offset relative to the tag being tracked.
     * @param fieldRelativeOffset The offset relative to the field. This is flipped to be alliance-relative.
     * @param finishSequence If present, the close lineup waits for this to be true before finishing. If not present,
     *            the close lineup finishes when the target is fully aligned.
     * @param lineupFeedback A callback that is called every loop which can be used to provide feedback to the driver in
     *            teleop for when the robot is properly aligned. Can be null.
     */
    public CloseLineupCommand(Drive drive, Vision vision, LEDs leds, int tagToTrack, Transform2d tagRelativeOffset,
        Supplier<Transform2d> fieldRelativeOffset, Optional<BooleanSupplier> finishSequence,
        BooleanConsumer lineupFeedback) {
        this.drive = drive;
        this.vision = vision;
        this.leds = leds;

        this.tagToTrack = tagToTrack;

        this.tagRelativeOffset = tagRelativeOffset;
        this.fieldRelativeOffset = fieldRelativeOffset;

        var xController = new PIDController(translationKp.get(), translationKi.get(), translationKd.get());
        var yController = new PIDController(translationKp.get(), translationKi.get(), translationKd.get());
        var thetaController = new ProfiledPIDController(thetaRotationKp.get(), thetaRotationKi.get(),
            thetaRotationKd.get(), new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
        thetaController.setIZone(Units.degreesToRadians(thetaIZone.get()));

        driveController = new HolonomicDriveController(xController, yController, thetaController);
        driveController.setTolerance(new Pose2d(Units.inchesToMeters(translationTolerance.get()),
            Units.inchesToMeters(translationTolerance.get()), Rotation2d.fromDegrees(thetaTolerance.get())));

        this.finishSequence = finishSequence;
        this.lineupFeedback = lineupFeedback;

        addRequirements(drive);
        setName("CloseLineup");
    }

    @Override
    public void initialize() {
        inRadiusDeadband = false;
        inThetaDeadband = false;
        driveController.getThetaController().reset(drive.getRotation().getRadians());
    }

    private boolean atSetpoint() {
        return driveController.atReference();
    }

    private Transform2d flipFieldTransform(Transform2d transform) {
        return new Transform2d(new Translation2d(-transform.getX(), -transform.getY()),
            transform.getRotation().unaryMinus());
    }

    @Override
    @SuppressWarnings("unused")
    public void execute() {
        LoggedTunableNumber.ifChanged(hashCode(), (double[] values) -> {
            driveController.getXController().setPID(values[0], values[1], values[2]);
            driveController.getYController().setPID(values[0], values[1], values[2]);
        }, translationKp, translationKi, translationKd);
        LoggedTunableNumber.ifChanged(hashCode(), (double[] values) -> {
            driveController.getThetaController().setPID(values[0], values[1], values[2]);
        }, thetaRotationKp, thetaRotationKi, thetaRotationKd);
        LoggedTunableNumber.ifChanged(hashCode(), (double[] values) -> {
            driveController.setTolerance(new Pose2d(Units.inchesToMeters(translationTolerance.get()),
                Units.inchesToMeters(translationTolerance.get()), Rotation2d.fromDegrees(thetaTolerance.get())));
        }, translationTolerance, thetaTolerance);
        LoggedTunableNumber.ifChanged(hashCode(), (double[] values) -> {
            driveController.getThetaController().setIZone(Units.degreesToRadians(values[0]));
        }, thetaIZone);
        LoggedTunableNumber.ifChanged(hashCode(), (double[] values) -> {
            driveController.getThetaController().setConstraints(new TrapezoidProfile.Constraints(values[0], values[1]));
        }, maxVelocity, maxAcceleration);

        Pose2d correctedCurrentPose = drive.getPose();
        Transform3d robotToTag = useSingleTag.getAsBoolean() ? vision.getRobotToTag(tagToTrack) : null;
        Pose2d fieldTagPose = VisionConstants.aprilTagLayout.getTagPose(tagToTrack).get().toPose2d();

        Logger.recordOutput("CloseLineup/UsingSingleTag", robotToTag != null);
        if(robotToTag != null) {
            Transform2d robotToTag2d = new Transform2d(robotToTag.getTranslation().toTranslation2d(),
                robotToTag.getRotation().toRotation2d());
            correctedCurrentPose = fieldTagPose.plus(robotToTag2d.inverse());
        }

        Logger.recordOutput("CloseLineup/CorrectedSingleTagPose", correctedCurrentPose);

        Transform2d absoluteOffset = fieldRelativeOffset.get();
        if(AutoBuilder.shouldFlip()) {
            absoluteOffset = flipFieldTransform(absoluteOffset);
        }

        Pose2d currentTargetPose = fieldTagPose.transformBy(tagRelativeOffset);
        currentTargetPose = new Pose2d(currentTargetPose.getTranslation().plus(absoluteOffset.getTranslation()),
            currentTargetPose.getRotation().plus(absoluteOffset.getRotation()));

        Logger.recordOutput("CloseLineup/TargetPose", currentTargetPose);

        ChassisSpeeds speed = driveController.calculate(correctedCurrentPose, currentTargetPose, 0,
            currentTargetPose.getRotation());

        boolean atSetpoint = atSetpoint();
        if(lineupFeedback != null) {
            lineupFeedback.accept(feedbackDebouncer.calculate(atSetpoint));
        }
        leds.setStateActive(LEDState.AutoScoreReady, ledDebouncer.calculate(atSetpoint));

        double radiusMagnitude = Math.sqrt(
            speed.vxMetersPerSecond * speed.vxMetersPerSecond + speed.vyMetersPerSecond * speed.vyMetersPerSecond);

        if(!inRadiusDeadband && radiusMagnitude < radiusInnerDeadband.get()
            && driveController.atReference()) inRadiusDeadband = true;
        if(inRadiusDeadband && radiusMagnitude > radiusOuterDeadband.get()) inRadiusDeadband = false;

        if(!inThetaDeadband && Math.abs(speed.omegaRadiansPerSecond) < Units.degreesToRadians(thetaInnerDeadband.get())
            && driveController.atReference()) inThetaDeadband = true;
        if(inThetaDeadband && Math.abs(speed.omegaRadiansPerSecond) > Units
            .degreesToRadians(thetaOuterDeadband.get())) inThetaDeadband = false;

        if(inRadiusDeadband) {
            speed.vxMetersPerSecond = 0;
            speed.vyMetersPerSecond = 0;
        }
        if(inThetaDeadband) {
            speed.omegaRadiansPerSecond = 0;
        }

        drive.runVelocity(speed);
    }

    @Override
    public boolean isFinished() {
        if(finishSequence.isEmpty()) {
            if(atSetpointDebouncer.calculate(atSetpoint())) return true;
        } else {
            if(finishSequence.get().getAsBoolean()) return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        leds.setStateActive(LEDState.AutoScoreReady, false);
        drive.runVelocity(new ChassisSpeeds(0, 0, 0));
    }
}
