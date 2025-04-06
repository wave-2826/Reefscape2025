package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefBranch;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.commands.AutoScoreCommands;
import frc.robot.commands.LoggedCommand;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.pieceVision.PieceVision;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.Container;
import frc.robot.util.ReefTarget;

public class AutoCommands {
    private static boolean grabbingCoralFailed = false;

    public static void registerNamedCommands(Drive drive, Vision vision, PieceVision pieceVision, Arm arm,
        Intake intake, LEDs leds) {
        // NamedCommands.registerCommand("Auto Coral", Commands
        //     .defer(() -> grabCoral(drive, pieceVision, intake, arm, null), Set.of(drive, pieceVision, intake, arm)));

        // TODO: Compute these ahead of time and don't use deferred commands
        // It seems this requires fixing some issues with the commands referencing alliances before they're run
        for(var branch : ReefBranch.values()) {
            for(var level : ReefLevel.values()) {
                ReefTarget target = new ReefTarget(branch, level);
                // @formatter:off
                registerLoggedNamedCommand("Wait/Score " + branch.toString() + " " + level.toString(),
                    Commands.defer(() -> Commands.sequence(
                        drive.runOnce(drive::stop),
                        Commands.waitUntil(() -> !IntakeCommands.waitingForPiece).withTimeout(2.5),
                        AutoScoreCommands.autoScoreCommand(drive, vision, arm, leds, target, true)
                            .onlyIf(() -> !IntakeCommands.waitingForPiece)
                    ), Set.of(drive, vision, arm))
                );
                registerLoggedNamedCommand("Score " + branch.toString() + " " + level.toString(), Commands.defer(() ->
                    AutoScoreCommands.autoScoreCommand(drive, vision, arm, leds, target, true),
                    Set.of(drive, vision, arm)
                ));
                
                registerLoggedNamedCommand("Fast Wait/Score " + branch.toString() + " " + level.toString(),
                    Commands.defer(() -> Commands.sequence(
                        drive.runOnce(drive::stop),
                        Commands.waitUntil(() -> !IntakeCommands.waitingForPiece).withTimeout(1.5),
                        AutoScoreCommands.autoScoreCommand(drive, vision, arm, leds, target, false)
                            .onlyIf(() -> !IntakeCommands.waitingForPiece)
                    ), Set.of(drive, vision, arm))
                );
                registerLoggedNamedCommand("Fast Score " + branch.toString() + " " + level.toString(), Commands.defer(() ->
                    AutoScoreCommands.autoScoreCommand(drive, vision, arm, leds, target, false),
                    Set.of(drive, vision, arm)
                ));
                // @formatter:on
            }
        }

        registerLoggedNamedCommand("Score Until Failure",
            Commands.defer(() -> scoreUntilFailure(drive, vision, arm, pieceVision, intake, leds),
                Set.of(drive, vision, arm, pieceVision, intake)));

        registerLoggedNamedCommand("Start intake",
            new ScheduleCommand(Commands.sequence(Commands.runOnce(() -> IntakeCommands.waitingForPiece = true),
                IntakeCommands.autoIntake(intake, arm))));
        registerLoggedNamedCommand("Intake That John", intakeThatJohn(drive, intake));

        registerLoggedNamedCommand("Prep arm",
            new ScheduleCommand(Commands.sequence(arm.goToStateCommand(ArmConstants.prepForScoringState, 1.0),
                Commands.runOnce(arm::resetToAbsolute))));
    }

    private static void registerLoggedNamedCommand(String name, Command command) {
        NamedCommands.registerCommand(name, new LoggedCommand(name, command));
    }

    /**
     * Constructs a command that repeatedly finds coral and scores it on the next available branch until it fails.
     */
    public static Command scoreUntilFailure(Drive drive, Vision vision, Arm arm, PieceVision pieceVision, Intake intake,
        LEDs leds) {
        List<ReefTarget> scoringPositionsAvailable = new ArrayList<>();
        return Commands.sequence(Commands.runOnce(() -> {
            grabbingCoralFailed = false;
            scoringPositionsAvailable.clear();
            var bluePose = AutoBuilder.shouldFlip() ? FlippingUtil.flipFieldPose(drive.getPose()) : drive.getPose();

            if(bluePose.getY() > FieldConstants.fieldWidth / 2.) {
                // If on the left side of the field
                scoringPositionsAvailable.add(new ReefTarget(ReefBranch.J, ReefLevel.L4));
                scoringPositionsAvailable.add(new ReefTarget(ReefBranch.K, ReefLevel.L4));
                scoringPositionsAvailable.add(new ReefTarget(ReefBranch.L, ReefLevel.L4)); // 4-piece auton? Probably not...
                scoringPositionsAvailable.add(new ReefTarget(ReefBranch.A, ReefLevel.L4)); // 5-piece auton!?! lol yeah right...   
            } else {
                // If on the right side of the field
                scoringPositionsAvailable.add(new ReefTarget(ReefBranch.E, ReefLevel.L4));
                scoringPositionsAvailable.add(new ReefTarget(ReefBranch.D, ReefLevel.L4));
                scoringPositionsAvailable.add(new ReefTarget(ReefBranch.C, ReefLevel.L4)); // 4-piece auton? Probably not...
                scoringPositionsAvailable.add(new ReefTarget(ReefBranch.B, ReefLevel.L4)); // 5-piece auton!?! lol yeah right...
            }
        }), Commands.sequence( //
            grabCoral(drive, pieceVision, intake, arm, () -> grabbingCoralFailed = true), //
            IntakeCommands.waitForPieceInArm().withTimeout(1.5),
            // If we don't have a piece, we failed to grab it
            Commands.runOnce(() -> {
                if(!IntakeCommands.waitingForPiece) {
                    grabbingCoralFailed = true;
                }
            }), //
            Commands.defer(() -> {
                var nextAvailableTarget = scoringPositionsAvailable.remove(0);
                if(scoringPositionsAvailable.isEmpty()) {
                    grabbingCoralFailed = true;
                    // Should never happen... 
                }

                return AutoScoreCommands.autoScoreCommand(drive, vision, arm, leds, nextAvailableTarget, true);
            }, Set.of(drive, vision, arm)).unless(() -> grabbingCoralFailed) //
        ).repeatedly().until(() -> grabbingCoralFailed)).withName("ScoreUntilFailure");
    }

    public static Command intakeThatJohn(Drive drive, Intake intake) {
        // @formatter:off
        Container<Pose2d> endPose = new Container<>(Pose2d.kZero);
        Container<Rotation2d> angle = new Container<>(Rotation2d.kZero);
        Container<Boolean> isLeft = new Container<>(false);
        double moveSpeedMPS = Units.feetToMeters(6);
        return Commands.sequence(
            Commands.runOnce(() -> {
                var bluePose = AutoBuilder.shouldFlip() ? FlippingUtil.flipFieldPose(drive.getPose()) : drive.getPose();

                isLeft.value = bluePose.getY() > FieldConstants.fieldWidth / 2.;
                angle.value = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red ? Rotation2d.kZero : Rotation2d.k180deg;
                endPose.value = new Pose2d(drive.getPose().getTranslation(), angle.value.rotateBy(Rotation2d.fromDegrees(180 + (isLeft.value ? -20 : 20))));
            }),
            // TODO: Mirror this rotation when on the other side of the field
            DriveCommands.driveStraightCommand(
                drive, moveSpeedMPS,
                () -> angle.value, () -> angle.value.rotateBy(Rotation2d.fromDegrees(180 + (isLeft.value ? 20 : -20)))
            ).until(() -> {
                if(intake.intakeSensorTriggered()) return true;

                // If the robot is at risk of running into the wall, stop.
                var robotPosition = drive.getPose();
                var nextPosition = robotPosition.exp(drive.getChassisSpeeds().toTwist2d(0.75));
                if(nextPosition.getX() < 0.0 || nextPosition.getY() < 0.0
                    || nextPosition.getX() > FieldConstants.fieldLength
                    || nextPosition.getY() > FieldConstants.fieldWidth) {
                    return true;
                }
                
                return false;
            }).withTimeout(3.0),
            Commands.defer(() -> new SimplePIDLineupCommand(drive, endPose.value), Set.of(drive)).withTimeout(2.0)
        );
        // @formatter:on
    }

    /**
     * Constructs a command that autonomously gets a coral piece and returns to where it started.
     * @return
     */
    public static Command grabCoral(Drive drive, PieceVision pieceVision, Intake intake, Arm arm,
        Runnable grabbingFailed) {
        Container<Pose2d> startPose = new Container<>(new Pose2d());
        return Commands.sequence( //
            Commands.runOnce(() -> {
                Logger.recordOutput("Auto/GrabbingCoral", true);
                // Record our starting position
                startPose.value = AutoBuilder.getCurrentPose();
            }),

            // Go to the coral piece
            GetCoralCommand.getCoral(pieceVision, drive, intake, arm, grabbingFailed),

            // Go back to our starting position
            new SimplePIDLineupCommand(drive, startPose.value),

            Commands.runOnce(() -> {
                Logger.recordOutput("Auto/GrabbingCoral", false);
            }) //
        ).withName("GrabCoral");
    }
}
