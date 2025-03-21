package frc.robot.commands.intake;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Controls.OperatorMode;
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

    public static Command intakeCommand(Intake intake, Arm arm, BooleanSupplier down
    // DoubleSupplier overrideSpeed,
    //     DoubleSupplier overridePitch, Supplier<OperatorMode> operatorMode
    ) {
        // @formatter:off
        return intake.run(() -> {
            // var mode = operatorMode.get();

            if(intake.intakeSensorTriggered()) {
                canTake = true;
            }

            // if(mode == OperatorMode.Normal) {
                intake.setIntakeState(down.getAsBoolean() ? IntakeState.Down : IntakeState.Up);

                if(intake.pieceWaitingForArm() && canTake) {
                    getPieceFromIntake(arm).schedule();
                    canTake = false;
                }
            // } else {
            //     intake.overrideIntakeSpeed(overrideSpeed.getAsDouble());
            //     intake.overridePitchPower(overridePitch.getAsDouble());
            // }
        }).withName("IntakeSequence");
    }
}
