package frc.robot.commands.drive;

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

    private final static LoggedTunableNumber translationKp = new LoggedTunableNumber("CloseLineup/translationKp");
    private final static LoggedTunableNumber translationKi = new LoggedTunableNumber("CloseLineup/translationKi");
    private final static LoggedTunableNumber translationKd = new LoggedTunableNumber("CloseLineup/translationKd");

    private final static LoggedTunableNumber thetaRotationKp = new LoggedTunableNumber("CloseLineup/thetaRotationKp");
    private final static LoggedTunableNumber thetaRotationKi = new LoggedTunableNumber("CloseLineup/thetaRotationKi");
    private final static LoggedTunableNumber thetaRotationKd = new LoggedTunableNumber("CloseLineup/thetaRotationKd");

    private final static LoggedTunableNumber xTranslationTolerance = new LoggedTunableNumber(
        "CloseLineup/xTranslationTolerance");
    private final static LoggedTunableNumber yTranslationTolerance = new LoggedTunableNumber(
        "CloseLineup/yTranslationTolerance");
    private final static LoggedTunableNumber thetaRotationTolerance = new LoggedTunableNumber(
        "CloseLineup/thetaRotationTolerance");

    static {
        translationKp.initDefault(1.75);
        translationKi.initDefault(0.0);
        translationKd.initDefault(0.0);

        thetaRotationKp.initDefault(1.5);
        thetaRotationKi.initDefault(0.0);
        thetaRotationKd.initDefault(0.0);

        xTranslationTolerance.initDefault(Units.inchesToMeters(0.75));
        yTranslationTolerance.initDefault(Units.inchesToMeters(0.75));
        thetaRotationTolerance.initDefault(Units.degreesToRadians(3));
    }

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
     */
    public CloseLineupCommand(Drive drive, Vision vision, int tagToTrack, Transform2d tagRelativeOffset,
        Supplier<Transform2d> fieldRelativeOffset) {
        this.drive = drive;
        this.vision = vision;
        this.tagToTrack = tagToTrack;

        this.tagRelativeOffset = tagRelativeOffset;
        this.fieldRelativeOffset = fieldRelativeOffset;

        xController = new PIDController(translationKp.get(), translationKi.get(), translationKd.get());
        yController = new PIDController(translationKp.get(), translationKi.get(), translationKd.get());
        thetaController = new PIDController(thetaRotationKp.get(), thetaRotationKi.get(), thetaRotationKd.get());
        thetaController.enableContinuousInput(0.0, Math.PI * 2);

        xController.setTolerance(xTranslationTolerance.get());
        yController.setTolerance(yTranslationTolerance.get());
        thetaController.setTolerance(thetaRotationTolerance.get());

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        thetaController.reset();
    }

    Transform2d flipFieldTransform(Transform2d transform) {
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
            xController.setTolerance(values[0]);
            yController.setTolerance(values[1]);
            thetaController.setTolerance(values[2]);
        }, xTranslationTolerance, yTranslationTolerance, thetaRotationTolerance);

        Pose2d correctedCurrentPose = drive.getPose();
        Transform3d robotToTag = vision.getRobotToTag(tagToTrack);
        Pose2d fieldTagPose = VisionConstants.aprilTagLayout.getTagPose(tagToTrack).get().toPose2d();
        if(robotToTag != null) {
            Transform2d robotToTag2d = new Transform2d(robotToTag.getTranslation().toTranslation2d(),
                robotToTag.getRotation().toRotation2d());
            correctedCurrentPose = fieldTagPose.plus(robotToTag2d);
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

        Logger.recordOutput("CloseLineup/TargetX", currentTargetPose.getX());
        Logger.recordOutput("CloseLineup/TargetY", currentTargetPose.getY());
        Logger.recordOutput("CloseLineup/TargetRotation", currentTargetPose.getRotation());

        double xSpeed = xController.calculate(correctedCurrentPose.getX());
        double ySpeed = yController.calculate(correctedCurrentPose.getY());
        double thetaSpeed = thetaController.calculate(correctedCurrentPose.getRotation().getRadians());

        // Terrible solution. Do not copy.
        if(xController.atSetpoint()) xSpeed = 0.;
        if(yController.atSetpoint()) ySpeed = 0.;

        ChassisSpeeds wheelSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed,
            correctedCurrentPose.getRotation());

        System.out.println("xSpeed: " + xSpeed + ", ySpeed: " + ySpeed + ", thetaSpeed: " + thetaSpeed);

        drive.runVelocity(wheelSpeeds);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.runVelocity(new ChassisSpeeds(0, 0, 0));
    }
}
