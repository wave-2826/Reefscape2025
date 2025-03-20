package frc.robot.commands.auto;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;

public class SimplePIDLineupCommand extends Command {
    private final Drive drive;

    private final PIDController xController, yController;
    private final PIDController thetaController;

    private final Pose2d targetPose;

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
        translationKp.initDefault(1.5);
        translationKi.initDefault(0.0);
        translationKd.initDefault(0.25);

        thetaRotationKp.initDefault(1.75);
        thetaRotationKi.initDefault(0.0);
        thetaRotationKd.initDefault(0.0);

        xTranslationTolerance.initDefault(Units.inchesToMeters(3.0));
        yTranslationTolerance.initDefault(Units.inchesToMeters(3.0));
        thetaRotationTolerance.initDefault(Units.degreesToRadians(20));
    }

    public SimplePIDLineupCommand(Drive drive, Pose2d pose) {
        this.drive = drive;
        xController = new PIDController(translationKp.get(), translationKi.get(), translationKd.get());
        yController = new PIDController(translationKp.get(), translationKi.get(), translationKd.get());
        thetaController = new PIDController(thetaRotationKp.get(), thetaRotationKi.get(), thetaRotationKd.get());
        thetaController.enableContinuousInput(0.0, Math.PI * 2);

        xController.setTolerance(xTranslationTolerance.get());
        yController.setTolerance(yTranslationTolerance.get());
        thetaController.setTolerance(thetaRotationTolerance.get());

        targetPose = pose;

        addRequirements(drive);
        setName("SimplePIDLineup");
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

        xController.setSetpoint(targetPose.getX());
        yController.setSetpoint(targetPose.getY());
        thetaController.setSetpoint(targetPose.getRotation().getRadians());

        Logger.recordOutput("Auto/CloseLineup/TargetPose", targetPose);

        Pose2d currentPose = drive.getPose();

        double xSpeed = xController.calculate(currentPose.getX());
        double ySpeed = yController.calculate(currentPose.getY());
        double thetaSpeed = thetaController.calculate(currentPose.getRotation().getRadians());

        ChassisSpeeds wheelSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed,
            currentPose.getRotation());

        drive.runVelocity(wheelSpeeds);
    }

    @Override
    public boolean isFinished() {
        return atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drive.runVelocity(new ChassisSpeeds(0, 0, 0));
    }
}
