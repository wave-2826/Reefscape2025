package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;

public class CloseLineupCommand extends Command {
    private final Drive drive;
    private Pose2d poseFinal, currentPose;

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
        translationKp.initDefault(3.0);
        translationKi.initDefault(0.0);
        translationKd.initDefault(0.0);

        thetaRotationKp.initDefault(2.0);
        thetaRotationKi.initDefault(0.0);
        thetaRotationKd.initDefault(0.0);

        xTranslationTolerance.initDefault(0.005);
        yTranslationTolerance.initDefault(0.005);
        thetaRotationTolerance.initDefault(Units.degreesToRadians(2));
    }

    public CloseLineupCommand(Drive drive, Pose2d finalPose) {
        this.drive = drive;
        this.poseFinal = finalPose;
        this.xController = new PIDController(translationKp.get(), translationKi.get(), translationKd.get());
        this.yController = new PIDController(translationKp.get(), translationKi.get(), translationKd.get());
        this.thetaController = new PIDController(thetaRotationKp.get(), thetaRotationKi.get(), thetaRotationKd.get());

        xController.setTolerance(xTranslationTolerance.get());
        yController.setTolerance(yTranslationTolerance.get());
        thetaController.setTolerance(thetaRotationTolerance.get());

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        currentPose = drive.getPose();

        xController.reset();
        yController.reset();
        thetaController.reset();

        xController.setSetpoint(poseFinal.getX());
        yController.setSetpoint(poseFinal.getY());
        thetaController.setSetpoint(poseFinal.getRotation().getRadians());
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

        currentPose = drive.getPose();

        double xSpeed = xController.calculate(currentPose.getX());
        double ySpeed = yController.calculate(currentPose.getY());
        double thetaSpeed = thetaController.calculate(currentPose.getRotation().getRadians());

        ChassisSpeeds wheelSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
        drive.runVelocity(wheelSpeeds);
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drive.runVelocity(new ChassisSpeeds(0, 0, 0));
        xController.close();
        yController.close();
        thetaController.close();
    }
}
