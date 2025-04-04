package frc.robot.commands.auto;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.pieceVision.PieceVision;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.Container;
import frc.robot.FieldConstants;
import frc.robot.commands.intake.IntakeCommands;

public class GetCoralCommand {
    private static final LoggedTunableNumber translationControllerP = new LoggedTunableNumber(//
        "GetCoral/xtranslationControllerP", 7.0);
    private static final LoggedTunableNumber translationControllerI = new LoggedTunableNumber(//
        "GetCoral/xtranslationControllerI", 0.0);
    private static final LoggedTunableNumber translationControllerD = new LoggedTunableNumber(//
        "GetCoral/xtranslationControllerD", 0.5);

    private static final LoggedTunableNumber thetaControllerP = new LoggedTunableNumber(//
        "GetCoral/thetaControllerP", 5.0);
    private static final LoggedTunableNumber thetaControllerI = new LoggedTunableNumber(//
        "GetCoral/thetaControllerI", 0.0);
    private static final LoggedTunableNumber thetaControllerD = new LoggedTunableNumber(//
        "GetCoral/thetaControllerD", 0.5);

    public static final LoggedTunableNumber lineFollowSpeed = new LoggedTunableNumber(//
        "GetCoral/lineFollowSpeed", DriveConstants.maxSpeedMetersPerSec * 0.7);

    public static Command getCoral(PieceVision pieceVision, Drive drive, Intake intake, Arm arm,
        Runnable grabbingFailed) {
        // A PID controller for the X position of the robot relative to the line.
        try(PIDController xController = new PIDController(translationControllerP.get(), translationControllerI.get(),
            translationControllerD.get());
            PIDController yController = new PIDController(translationControllerP.get(), translationControllerI.get(),
                translationControllerD.get());
            PIDController thetaController = new PIDController(thetaControllerP.get(), thetaControllerI.get(),
                thetaControllerD.get())) {
            Container<Double> startTime = new Container<>(0.0);
            Debouncer targetLineMissingDebouncer = new Debouncer(0.5, DebounceType.kRising);

            return Commands.parallel(//
                IntakeCommands.intakeCommand(intake, arm, () -> true, () -> false, () -> false), //
                Commands.sequence( //
                    Commands.runOnce(() -> {
                        xController.reset();
                        yController.reset();

                        xController.setSetpoint(0);
                        yController.setSetpoint(0);

                        targetLineMissingDebouncer.calculate(false);

                        startTime.value = Timer.getTimestamp();
                    }), //
                    Commands.run(() -> {
                        // Get the target line from the piece vision.
                        var targetLineMaybe = pieceVision.getTargetPath();

                        // If there's no target line, wait.
                        if(targetLineMaybe.isEmpty()) return;
                        var targetLine = targetLineMaybe.get();

                        var positionOnPath = (Timer.getTimestamp() - startTime.value) * lineFollowSpeed.get();

                        // Get the robot's position relative to the line.
                        var robotPosition = drive.getPose();
                        var linePosition = targetLine.getPoseAtDistance(positionOnPath);
                        Logger.recordOutput("Auto/GetCoralTargetPose", linePosition);

                        var robotToLine = robotPosition.relativeTo(linePosition);

                        var xError = robotToLine.getX();
                        var yError = robotToLine.getY();
                        var thetaError = robotToLine.getRotation().getRadians();

                        var xSpeed = -xController.calculate(xError);
                        var ySpeed = -yController.calculate(yError);
                        var thetaSpeed = -thetaController.calculate(thetaError);

                        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed,
                            robotPosition.getRotation()));
                    }, drive).until(() -> {
                        if(intake.intakeSensorTriggered()) return true;

                        if(targetLineMissingDebouncer.calculate(pieceVision.getTargetPath().isEmpty())) {
                            if(grabbingFailed != null) grabbingFailed.run();
                            return true;
                        }

                        // If the robot is at risk of running into the wall, stop.
                        var robotPosition = drive.getPose();
                        var nextPosition = robotPosition.exp(drive.getChassisSpeeds().toTwist2d(0.3));
                        if(nextPosition.getX() < 0.0 || nextPosition.getY() < 0.0
                            || nextPosition.getX() > FieldConstants.fieldLength
                            || nextPosition.getY() > FieldConstants.fieldWidth) {
                            if(grabbingFailed != null) grabbingFailed.run();
                            return true;
                        }
                        return false;
                    }).finallyDo(() -> {
                        drive.stop();
                    }) //
                ));
        }
    }
}
