package frc.robot.commands.drive;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.LoggedTunableNumber;

public class CloseLineupCommand extends Command {
    private final Drive drive;
    private final Vision vision;
    private final int tagToTrack;

    private final Transform2d tagRelativeOffset;
    private final Supplier<Transform2d> fieldRelativeOffset;

    private final PIDController xController, yController;
    private final PIDController thetaController;

    private final Optional<BooleanSupplier> finishSequence;
    private final BooleanConsumer lineupFeedback;

    private final static LoggedTunableNumber translationKp = new LoggedTunableNumber("CloseLineup/translationKp", 4.5);
    private final static LoggedTunableNumber translationKi = new LoggedTunableNumber("CloseLineup/translationKi", 0.0);
    private final static LoggedTunableNumber translationKd = new LoggedTunableNumber("CloseLineup/translationKd", 0.5);

    private final static LoggedTunableNumber thetaRotationKp = new LoggedTunableNumber("CloseLineup/thetaRotationKp",
        6.0);
    private final static LoggedTunableNumber thetaRotationKi = new LoggedTunableNumber("CloseLineup/thetaRotationKi",
        0.01);
    private final static LoggedTunableNumber thetaRotationKd = new LoggedTunableNumber("CloseLineup/thetaRotationKd",
        0.2);

    private final static LoggedTunableNumber xTranslationTolerance = new LoggedTunableNumber(
        "CloseLineup/xTranslationTolerance", 0.5);
    private final static LoggedTunableNumber yTranslationTolerance = new LoggedTunableNumber(
        "CloseLineup/yTranslationTolerance", 0.5);
    private final static LoggedTunableNumber thetaRotationTolerance = new LoggedTunableNumber(
        "CloseLineup/thetaRotationTolerance", 1.5);

    private final static LoggedTunableNumber xDerivativeTolerance = new LoggedTunableNumber(
        "CloseLineup/xTranslationTolerance", 0.5);
    private final static LoggedTunableNumber yDerivativeTolerance = new LoggedTunableNumber(
        "CloseLineup/yTranslationTolerance", 0.5);
    private final static LoggedTunableNumber thetaDerivativeTolerance = new LoggedTunableNumber(
        "CloseLineup/thetaRotationTolerance", 3);

    private final static LoggedTunableNumber thetaIZone = new LoggedTunableNumber("CloseLineup/thetaIZone", 1.0);

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
    public CloseLineupCommand(Drive drive, Vision vision, int tagToTrack, Transform2d tagRelativeOffset,
        Supplier<Transform2d> fieldRelativeOffset, Optional<BooleanSupplier> finishSequence,
        BooleanConsumer lineupFeedback) {
        this.drive = drive;
        this.vision = vision;
        this.tagToTrack = tagToTrack;

        this.tagRelativeOffset = tagRelativeOffset;
        this.fieldRelativeOffset = fieldRelativeOffset;

        xController = new PIDController(translationKp.get(), translationKi.get(), translationKd.get());
        yController = new PIDController(translationKp.get(), translationKi.get(), translationKd.get());
        thetaController = new PIDController(thetaRotationKp.get(), thetaRotationKi.get(), thetaRotationKd.get());
        thetaController.enableContinuousInput(0.0, Math.PI * 2);

        xController.setTolerance(Units.inchesToMeters(xTranslationTolerance.get()),
            Units.inchesToMeters(xDerivativeTolerance.get()));
        yController.setTolerance(Units.inchesToMeters(yTranslationTolerance.get()),
            Units.inchesToMeters(yDerivativeTolerance.get()));
        thetaController.setTolerance(Units.degreesToRadians(thetaRotationTolerance.get()),
            Units.degreesToRadians(thetaDerivativeTolerance.get()));

        thetaController.setIntegratorRange(-Units.degreesToRadians(thetaIZone.get()),
            Units.degreesToRadians(thetaIZone.get()));

        this.finishSequence = finishSequence;
        this.lineupFeedback = lineupFeedback;

        addRequirements(drive);
        setName("CloseLineup");
    }

    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        thetaController.reset();
    }

    private boolean atSetpoint() {
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }

    private Transform2d flipFieldTransform(Transform2d transform) {
        return new Transform2d(new Translation2d(-transform.getX(), transform.getY()),
            transform.getRotation().unaryMinus());
    }

    @Override
    public void execute() {
        LoggedTunableNumber.ifChanged(hashCode(), (double[] values) -> {
            xController.setPID(values[0], values[1], values[2]);
            yController.setPID(values[0], values[1], values[2]);
        }, translationKp, translationKi, translationKd);
        LoggedTunableNumber.ifChanged(hashCode(), (double[] values) -> {
            thetaController.setPID(values[0], values[1], values[2]);
        }, thetaRotationKp, thetaRotationKi, thetaRotationKd);
        LoggedTunableNumber.ifChanged(hashCode(), (double[] values) -> {
            xController.setTolerance(Units.inchesToMeters(values[0]));
            yController.setTolerance(Units.inchesToMeters(values[1]));
            thetaController.setTolerance(Units.degreesToRadians(values[2]));
        }, xTranslationTolerance, yTranslationTolerance, thetaRotationTolerance);
        LoggedTunableNumber.ifChanged(hashCode(), (double[] values) -> {
            thetaController.setIntegratorRange(-Units.degreesToRadians(values[0]), Units.degreesToRadians(values[0]));
        }, thetaIZone);

        Pose2d correctedCurrentPose = drive.getPose();
        Transform3d robotToTag = vision.getRobotToTag(tagToTrack);
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

        xController.setSetpoint(currentTargetPose.getX());
        yController.setSetpoint(currentTargetPose.getY());
        thetaController.setSetpoint(currentTargetPose.getRotation().getRadians());

        Logger.recordOutput("CloseLineup/TargetPose", currentTargetPose);

        double xSpeed = xController.calculate(correctedCurrentPose.getX());
        double ySpeed = yController.calculate(correctedCurrentPose.getY());
        double thetaSpeed = thetaController.calculate(correctedCurrentPose.getRotation().getRadians());

        // Terrible solution. Do not copy.
        if(xController.atSetpoint()) xSpeed = 0.;
        if(yController.atSetpoint()) ySpeed = 0.;

        if(lineupFeedback != null) {
            lineupFeedback.accept(atSetpoint());
        }

        ChassisSpeeds wheelSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed,
            correctedCurrentPose.getRotation());

        drive.runVelocity(wheelSpeeds);
    }

    @Override
    public boolean isFinished() {
        if(finishSequence.isEmpty()) {
            if(atSetpoint()) return true;
        } else {
            if(finishSequence.get().getAsBoolean()) return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.runVelocity(new ChassisSpeeds(0, 0, 0));
    }
}
