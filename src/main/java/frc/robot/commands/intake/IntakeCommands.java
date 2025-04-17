package frc.robot.commands.intake;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;

public class IntakeCommands {
    @AutoLogOutput(key = "Auto/WaitingForPiece")
    public static boolean waitingForPiece = false;

    public static Command waitForPieceInArm() {
        return Commands.sequence(Commands.runOnce(() -> waitingForPiece = true),
            Commands.waitUntil(() -> !waitingForPiece));
    }

    public static Command getPieceFromIntake(Arm arm) {
        return Commands.sequence(arm.goToStateCommand(ArmConstants.restingState),
            arm.goToStateCommand(ArmConstants.getPieceState, 0.2), Commands.waitSeconds(0.06),
            arm.goToStateCommand(ArmConstants.restingState, 0.2), Commands.runOnce(() -> waitingForPiece = false),
            arm.goToStateCommand(ArmConstants.prepForScoringState, 0.2).onlyIf(DriverStation::isTeleop));
    }

    @AutoLogOutput(key = "Intake/CanTake")
    private static boolean canTake = false;

    /**
     * The intake command intended for autonomous. It will run the intake down, wait for a piece to be detected, and
     * then run the arm to get the piece from the intake.
     * @param intake
     * @param arm
     * @return
     */
    public static Command autoIntake(Intake intake, Arm arm) {
        // @formatter:off
        return Commands.sequence(
            intake.runOnce(() -> {
                intake.setIntakeState(IntakeState.IntakeDown);
            }),
            Commands.waitSeconds(0.1),
            arm.goToStateCommand(ArmConstants.restingState),
            Commands.waitUntil(intake::pieceWaitingForArm),
            intake.runOnce(() -> {
                intake.setIntakeState(IntakeState.Up);
            }),
            new ProxyCommand(getPieceFromIntake(arm)) 
        ).withName("AutoIntakeSequence");
        // @formatter:on
    }

    /**
     * A command that runs the intake and arm to get a piece from the intake.
     * @param intake
     * @param arm
     * @param shouldIntake
     * @param shouldOuttake
     * @return
     */
    public static Command intakeCommand(Intake intake, Arm arm, BooleanSupplier shouldIntake,
        BooleanSupplier shouldOuttake) {
        // @formatter:off
        return intake.run(() -> {
            if(intake.intakeSensorTriggered() && !shouldOuttake.getAsBoolean()) {
                canTake = true;
            }

            if(shouldIntake.getAsBoolean()) {
                intake.setIntakeState(IntakeState.IntakeDown);
            } else if(shouldOuttake.getAsBoolean()) {
                intake.setIntakeState(IntakeState.OuttakeDown);
            } else {
                intake.setIntakeState(IntakeState.Up);
            }
            
            if(intake.pieceWaitingForArm() && canTake) {
                getPieceFromIntake(arm).schedule();
                canTake = false;
            }
        }).withName("IntakeSequence");
    }
}
