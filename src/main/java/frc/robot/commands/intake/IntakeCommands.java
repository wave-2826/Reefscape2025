package frc.robot.commands.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;

public class IntakeCommands {
    public static Command getPieceFromIntake(Arm arm) {
        return Commands.sequence(arm.goToStateCommand(ArmConstants.restingState),
            arm.goToStateCommand(ArmConstants.getPieceState), Commands.waitSeconds(0.1),
            arm.goToStateCommand(ArmConstants.restingState));
    }

    private static boolean canTake = false;

    public static Command intakeCommand(Intake intake, Arm arm, BooleanSupplier down) {
        // @formatter:off
        return intake.run(() -> {
            intake.setIntakeState(down.getAsBoolean() ? IntakeState.Down : IntakeState.Up);

            if(intake.intakeSensorTriggered()) {
                canTake = true;
            }
            if(intake.pieceWaitingForArm() && canTake) {
                getPieceFromIntake(arm).schedule();
                canTake = false;
            }
        }).withName("IntakeSequence");
    }
}
