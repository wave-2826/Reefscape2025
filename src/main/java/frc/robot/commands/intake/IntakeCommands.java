package frc.robot.commands.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Controls;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {
    public static Command intakeCommand(Intake intake, Arm arm) {
        // @formatter:off
        return Commands.sequence(
            intake.runIntakeOpenLoop(1.0),
            intake.setIntakePitch(Rotation2d.kZero),
            intake.setIntakeCoast(),

            Commands.waitUntil(intake::intakeSensorTriggered),   
            intake.setTransportOverrideSpeed(1.0),     

            Commands.runOnce(() -> {
                // This is kind of sketchy 
                Commands.sequence(
                    arm.goToStateCommand(ArmConstants.intakeClearanceState),
                    
                    Commands.waitUntil(intake::transportSensorTriggered),
                    Commands.waitSeconds(0.1),
                    intake.setTransportOverrideSpeed(0.0),

                    arm.goToStateCommand(ArmConstants.getPieceState),
                    Commands.waitSeconds(0.4),
                    arm.goToStateCommand(ArmConstants.restingState)
                ).schedule();
            }),

            Controls.getInstance().controllerRumbleWhileRunning(true, true, RumbleType.kBothRumble)
                .withTimeout(0.1)
                .andThen(Commands.waitSeconds(0.05))
                .repeatedly().withTimeout(0.4)
        ).finallyDo(() -> {
            Commands.sequence(
                intake.runIntakeOpenLoop(0.0),
                intake.setIntakePitch(Rotation2d.fromDegrees(45))
            ).schedule();
        });
        // @formatter:on
    }
}
