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

    private final static LoggedTunableNumber translationKp = new LoggedTunableNumber(//
        "SimplePIDLineup/translationKp", 7.5);
    private final static LoggedTunableNumber translationKi = new LoggedTunableNumber(//
        "SimplePIDLineup/translationKi", 0.0);
    private final static LoggedTunableNumber translationKd = new LoggedTunableNumber(//
        "SimplePIDLineup/translationKd", 1.25);

    private final static LoggedTunableNumber thetaRotationKp = new LoggedTunableNumber(//
        "SimplePIDLineup/thetaRotationKp", 7.0);
    private final static LoggedTunableNumber thetaRotationKi = new LoggedTunableNumber(//
        "SimplePIDLineup/thetaRotationKi", 0.0);
    private final static LoggedTunableNumber thetaRotationKd = new LoggedTunableNumber(//
        "SimplePIDLineup/thetaRotationKd", 0.75);

    private final static LoggedTunableNumber translationTolerance = new LoggedTunableNumber(//
        "SimplePIDLineup/translationTolerance", Units.inchesToMeters(3.0));
    private final static LoggedTunableNumber thetaRotationTolerance = new LoggedTunableNumber(//
        "SimplePIDLineup/thetaRotationTolerance", Units.degreesToRadians(10));

    public SimplePIDLineupCommand(Drive drive, Pose2d pose) {
        this.drive = drive;
        xController = new PIDController(translationKp.get(), translationKi.get(), translationKd.get());
        yController = new PIDController(translationKp.get(), translationKi.get(), translationKd.get());
        thetaController = new PIDController(thetaRotationKp.get(), thetaRotationKi.get(), thetaRotationKd.get());
        thetaController.enableContinuousInput(0.0, Math.PI * 2);

        xController.setTolerance(translationTolerance.get());
        yController.setTolerance(translationTolerance.get());
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
            yController.setTolerance(values[0]);
            thetaController.setTolerance(values[1]);
        }, translationTolerance, thetaRotationTolerance);

        xController.setSetpoint(targetPose.getX());
        yController.setSetpoint(targetPose.getY());
        thetaController.setSetpoint(targetPose.getRotation().getRadians());

        Logger.recordOutput("Auto/SimplePIDLineup/TargetPose", targetPose);

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
