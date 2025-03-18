package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.pieceVision.PieceVision;
import frc.robot.util.LoggedTunableNumber;

public class GetCoralCommand {
    private static final LoggedTunableNumber xControllerP = new LoggedTunableNumber("GetCoral/xControllerP");
    private static final LoggedTunableNumber xControllerI = new LoggedTunableNumber("GetCoral/xControllerI");
    private static final LoggedTunableNumber xControllerD = new LoggedTunableNumber("GetCoral/xControllerD");

    static {
        xControllerP.initDefault(2.0);
        xControllerI.initDefault(0);
        xControllerD.initDefault(0);
    }

    private static Command followTargetLine(PieceVision pieceVision, Drive drive) {
        // A PID controller for the X position of the robot relative to the line.
        PIDController xController = new PIDController(xControllerP.get(), xControllerI.get(), xControllerD.get());
        // PIDController 
        return Commands.run(() -> {
            // drive.runVelocity(ChassisSpeeds.)
        }, drive);
    }

    public static Command getCoral(PieceVision pieceVision, Drive drive) {
        // TODO
        return Commands.none();
    }
}
