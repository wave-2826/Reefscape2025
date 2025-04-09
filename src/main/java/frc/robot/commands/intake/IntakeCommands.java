package frc.robot.commands.intake;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
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
            arm.goToStateCommand(ArmConstants.restingState, 0.2),
            arm.goToStateCommand(ArmConstants.prepForScoringState, 0.2),
            Commands.runOnce(() -> waitingForPiece = false));
    }

    @AutoLogOutput(key = "Intake/CanTake")
    private static boolean canTake = false;

    public static Command autoIntake(Intake intake, Arm arm) {
        // @formatter:off
        return Commands.sequence(
            intake.runOnce(() -> {
                intake.setIntakeState(IntakeState.IntakeDown);
            }),
            Commands.waitUntil(intake::pieceWaitingForArm),
            new ScheduleCommand(getPieceFromIntake(arm)),
            intake.runOnce(() -> {
                intake.setIntakeState(IntakeState.Up);
            })
        ).withName("AutoIntakeSequence");
        // @formatter:on
    }

    public static Command intakeCommand(Intake intake, Arm arm, BooleanSupplier shouldIntake,
        BooleanSupplier shouldOuttake, BooleanSupplier shouldOuttakeTrough
    // DoubleSupplier overrideSpeed,
    //     DoubleSupplier overridePitch, Supplier<OperatorMode> operatorMode
    ) {
        // @formatter:off
        return intake.run(() -> {
            if(intake.intakeSensorTriggered() && !shouldOuttake.getAsBoolean() && !shouldOuttakeTrough.getAsBoolean()) {
                canTake = true;
            }

            if(shouldIntake.getAsBoolean()) {
                intake.setIntakeState(IntakeState.IntakeDown);
            } else if(shouldOuttake.getAsBoolean()) {
                intake.setIntakeState(IntakeState.OuttakeDown);
            } else if(shouldOuttakeTrough.getAsBoolean()) {
                intake.setIntakeState(IntakeState.OuttakeUp);
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
