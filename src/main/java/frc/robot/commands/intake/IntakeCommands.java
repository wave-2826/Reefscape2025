package frc.robot.commands.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Controls;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {
    public static Command getPiece(Arm arm) {
        return Commands.sequence(arm.goToStateCommand(ArmConstants.getPieceState).withTimeout(0.5),
            Commands.waitSeconds(0.8), arm.goToStateCommand(ArmConstants.getPieceState2).withTimeout(0.5),
            arm.goToStateCommand(ArmConstants.getPieceState).withTimeout(0.5),
            arm.goToStateCommand(ArmConstants.restingState).withTimeout(0.25));
    }

    public static Command getPieceFromIntake(Intake intake, Arm arm) {
        // @formatter:off
        return Commands.sequence(
            arm.goToStateCommand(ArmConstants.restingState).withTimeout(0.5),
            intake.setTransportOverrideSpeedCommand(1.0),

            Commands.sequence(
                Commands.waitUntil(intake::transportSensorTriggered),
                Commands.waitSeconds(0.15),
                intake.setTransportOverrideSpeedCommand(0.0),

                getPiece(arm)
            ).withTimeout(0.75)
        ).withName("GetPieceFromIntake");
        // @formatter:on
    }

    public static Command intakeCommand(Intake intake, Arm arm) {
        // @formatter:off
        return Commands.sequence(
            Commands.sequence(
                intake.runIntakeOpenLoopCommand(1.0),
                intake.setIntakePitchCommand(Rotation2d.kZero),
                intake.setIntakeCoast(),

                Commands.waitUntil(intake::intakeSensorTriggered)
            ).onlyIf(() -> !intake.intakeSensorTriggered()),

            Commands.runOnce(() -> {
                // This is kind of sketchy
                getPieceFromIntake(intake, arm).schedule();
            }),

            Controls.getInstance().controllerRumbleWhileRunning(true, true, RumbleType.kBothRumble)
                .withTimeout(0.1)
                .andThen(Commands.waitSeconds(0.05))
                .repeatedly().withTimeout(0.4)
                .onlyIf(DriverStation::isTeleop)
        ).finallyDo(() -> {
            intake.runIntakeOpenLoop(0.0);
            intake.setIntakePitchCommand(Rotation2d.fromDegrees(80)).schedule();
        })
        // HACK
        .withName("IntakeSequence");
        // @formatter:on
    }
}
