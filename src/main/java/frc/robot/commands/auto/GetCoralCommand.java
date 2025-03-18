package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
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
    private static final LoggedTunableNumber xControllerP = new LoggedTunableNumber("GetCoral/xControllerP");
    private static final LoggedTunableNumber xControllerI = new LoggedTunableNumber("GetCoral/xControllerI");
    private static final LoggedTunableNumber xControllerD = new LoggedTunableNumber("GetCoral/xControllerD");

    private static final LoggedTunableNumber yControllerP = new LoggedTunableNumber("GetCoral/yControllerP");
    private static final LoggedTunableNumber yControllerI = new LoggedTunableNumber("GetCoral/yControllerI");
    private static final LoggedTunableNumber yControllerD = new LoggedTunableNumber("GetCoral/yControllerD");

    private static final LoggedTunableNumber lineFollowSpeed = new LoggedTunableNumber("GetCoral/lineFollowSpeed");

    static {
        xControllerP.initDefault(2.0);
        xControllerI.initDefault(0);
        xControllerD.initDefault(0);

        yControllerP.initDefault(2.0);
        yControllerI.initDefault(0);
        yControllerD.initDefault(0);

        lineFollowSpeed.initDefault(DriveConstants.maxSpeedMetersPerSec * 0.7);
    }

    public static Command getCoral(PieceVision pieceVision, Drive drive, Intake intake, Arm arm) {
        // A PID controller for the X position of the robot relative to the line.
        try(PIDController xController = new PIDController(xControllerP.get(), xControllerI.get(), xControllerD.get());
            PIDController yController = new PIDController(yControllerP.get(), yControllerI.get(), yControllerD.get())) {
            Container<Double> startTime = new Container<>(0.0);

            return Commands.parallel(Commands.sequence( //
                Commands.runOnce(() -> {
                    xController.reset();
                    yController.reset();

                    xController.setTolerance(0.1);
                    yController.setTolerance(0.1);

                    xController.setSetpoint(0);
                    yController.setSetpoint(0);

                    startTime.value = Timer.getTimestamp();
                }), //
                Commands.run(() -> {
                    // Get the target line from the piece vision.
                    var targetLineMaybe = pieceVision.getTargetPath();

                    // If there's no target line, wait.
                    if(targetLineMaybe == null) { return; }
                    var targetLine = targetLineMaybe.get();

                    var positionOnPath = (startTime.value - Timer.getTimestamp()) * lineFollowSpeed.get();

                    // Get the robot's position relative to the line.
                    var robotPosition = drive.getPose();
                    var linePosition = targetLine.getPoseAtDistance(positionOnPath);
                    var robotToLine = linePosition.relativeTo(robotPosition);

                    var xError = robotToLine.getX();
                    var yError = robotToLine.getY();
                    var xSpeed = xController.calculate(xError);
                    var ySpeed = yController.calculate(yError);

                    drive.runVelocity(
                        ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed, ySpeed, 0, robotPosition.getRotation()));
                }, drive).until(() -> {
                    if(intake.intakeSensorTriggered()) return true;

                    // If the robot is at risk of running into the wall, stop.
                    var robotPosition = drive.getPose();
                    var nextPosition = robotPosition.exp(drive.getChassisSpeeds().toTwist2d(0.3));
                    return nextPosition.getX() < 0.0 || nextPosition.getY() < 0.0
                        || nextPosition.getX() > FieldConstants.fieldLength
                        || nextPosition.getY() > FieldConstants.fieldWidth;
                }).finallyDo(() -> {
                    drive.stop();
                }) //
            ), IntakeCommands.intakeCommand(intake, arm));
        }
    }
}
