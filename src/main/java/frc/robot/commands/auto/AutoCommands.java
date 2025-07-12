package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefBranch;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.RobotState;
import frc.robot.commands.AutoScoreCommands;
import frc.robot.commands.LoggedCommand;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.util.Container;
import frc.robot.util.Elastic;
import frc.robot.util.ReefTarget;

public class AutoCommands {
    private static boolean grabbingCoralFailed = false;

    public static void registerNamedCommands(Drive drive, Arm arm, Intake intake, LEDs leds) {
        // NamedCommands.registerCommand("Auto Coral", Commands
        //     .defer(() -> grabCoral(drive, pieceVision, intake, arm, null), Set.of(drive, pieceVision, intake, arm)));

        // TODO: Compute these ahead of time and don't use deferred commands
        // It seems this requires fixing some issues with the commands referencing alliances before they're run
        for(var branch : ReefBranch.values()) {
            for(var level : ReefLevel.values()) {
                ReefTarget target = new ReefTarget(branch, level);
                // @formatter:off
                registerLoggedNamedCommand("Score " + branch.toString() + " " + level.toString(), Commands.defer(() ->
                    AutoScoreCommands.autoScoreCommand(drive, arm, leds, target, true),
                    Set.of(drive, arm)
                ));
                
                registerLoggedNamedCommand("Fast Score " + branch.toString() + " " + level.toString(), Commands.defer(() ->
                    AutoScoreCommands.autoScoreCommand(drive, arm, leds, target, false),
                    Set.of(drive, arm)
                ));
                // @formatter:on
            }
        }

        registerLoggedNamedCommand("Auto Coral Vision Score",
            Commands.defer(() -> scoreUntilFailure(drive, arm, intake, leds), Set.of(drive, arm, intake)));

        // @formatter:off
        registerLoggedNamedCommand("Gamble Wait", Commands.sequence(
            Commands.waitSeconds(1.5),
            new ScheduleCommand(arm.goToStateCommand(ArmConstants.restingState))
        ));
        // @formatter:on

        Container<Pose2d> startPosition = new Container<>(Pose2d.kZero);
        final double MAX_VALID_GAMBLE_DISTANCE = 3.0;
        registerLoggedNamedCommand("Auto Grab Gamble Coral", Commands.sequence(//
            new ScheduleCommand(IntakeCommands.autoIntake(intake, arm)).beforeStarting(() -> {
                RobotState.getInstance().resetCoralPositions();
                IntakeCommands.waitingForPiece = true;
                grabbingCoralFailed = false;
                startPosition.value = RobotState.getInstance().getPose();
            }), //

            Commands.runOnce(() -> {
                Elastic.sendNotification(new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR,
                    "Piece vision camera disconnected!", "Autonomous routine will fail!"));
            }).onlyIf(() -> RobotState.getInstance().pieceVisionDisconnected),

            new LoggedCommand("Grab Gamble Coral",
                new TrackCoral(drive, leds, () -> grabbingCoralFailed = true, (pos) -> {
                    return startPosition.value.getTranslation().minus(pos).getNorm() > Units
                        .feetToMeters(MAX_VALID_GAMBLE_DISTANCE);
                }).until(() -> {
                    if(intake.intakeSensorTriggered()) return true;
                    if(startPosition.value.minus(RobotState.getInstance().getPose()).getTranslation().getNorm() > Units
                        .feetToMeters(MAX_VALID_GAMBLE_DISTANCE + 0.25)) {
                        grabbingCoralFailed = true;
                        return true;
                    }
                    return false;
                }).withTimeout(6.)) //
        ));

        registerLoggedNamedCommand("Wait For Gamble Coral", Commands.waitUntil(() -> !IntakeCommands.waitingForPiece));

        registerLoggedNamedCommand("Start intake", new ScheduleCommand(
            IntakeCommands.autoIntake(intake, arm).beforeStarting(() -> IntakeCommands.waitingForPiece = true)));
        registerLoggedNamedCommand("Intake That John", intakeThatJohn(drive, intake));

        registerLoggedNamedCommand("Prep arm", new ScheduleCommand(Commands.sequence(Commands.runOnce(() -> {
            IntakeCommands.waitingForPiece = false;
        }), arm.goToStateCommand(ArmConstants.prepForScoringState, 1.0))));
    }

    private static void registerLoggedNamedCommand(String name, Command command) {
        NamedCommands.registerCommand(name, new LoggedCommand(name, command));
    }

    /**
     * Constructs a command that repeatedly finds coral and scores it on the next available branch until it fails.
     */
    public static Command scoreUntilFailure(Drive drive, Arm arm, Intake intake, LEDs leds) {
        List<ReefTarget> scoringPositionsAvailable = new ArrayList<>();

        return Commands.sequence(Commands.runOnce(() -> {
            grabbingCoralFailed = false;
            scoringPositionsAvailable.clear();

            if(RobotState.getInstance().isOnRightSide()) {
                // If on the right side of the field
                scoringPositionsAvailable.add(new ReefTarget(ReefBranch.D, ReefLevel.L4));
                scoringPositionsAvailable.add(new ReefTarget(ReefBranch.C, ReefLevel.L4));
                scoringPositionsAvailable.add(new ReefTarget(ReefBranch.B, ReefLevel.L4));
                scoringPositionsAvailable.add(new ReefTarget(ReefBranch.A, ReefLevel.L4));
            } else {
                // If on the left side of the field
                scoringPositionsAvailable.add(new ReefTarget(ReefBranch.K, ReefLevel.L4));
                scoringPositionsAvailable.add(new ReefTarget(ReefBranch.L, ReefLevel.L4));
                scoringPositionsAvailable.add(new ReefTarget(ReefBranch.A, ReefLevel.L4));
                scoringPositionsAvailable.add(new ReefTarget(ReefBranch.B, ReefLevel.L4));
            }
        }), Commands.sequence( //
            new ScheduleCommand(IntakeCommands.autoIntake(intake, arm))
                .beforeStarting(() -> IntakeCommands.waitingForPiece = true), //

            Commands.runOnce(() -> {
                Elastic.sendNotification(new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR,
                    "Piece vision camera disconnected!", "Autonomous routine will fail!"));
            }).onlyIf(() -> RobotState.getInstance().pieceVisionDisconnected),

            new LoggedCommand("Grab Coral", new TrackCoral(drive, leds, () -> grabbingCoralFailed = true).until(() -> {
                if(intake.intakeSensorTriggered()) return true;

                // If the robot is at risk of running into the wall, stop.
                var intakeOrigin = new Transform2d(new Translation2d(Units.inchesToMeters(-16), 0), Rotation2d.kZero);
                var nextPosition = RobotState.getInstance().getLookaheadPose(0.2).transformBy(intakeOrigin);
                // TODO: Detect this as a polygon, not a rectangle. Don't run into the source!
                if(nextPosition.getX() < 0.0 || nextPosition.getY() < 0.0
                    || nextPosition.getX() > FieldConstants.fieldLength
                    || nextPosition.getY() > FieldConstants.fieldWidth) {
                    grabbingCoralFailed = true;
                    return true;
                }
                return false;
            }).withTimeout(6.)).unless(intake::pieceInTransport), //

            new LoggedCommand("AutoTargetScoring", Commands.defer(() -> {
                if(scoringPositionsAvailable.isEmpty()) {
                    // Should never happen... 
                    grabbingCoralFailed = true;
                    return Commands.runOnce(() -> drive.stop());
                }

                var nextAvailableTarget = scoringPositionsAvailable.remove(0);

                return AutoScoreCommands.autoScoreCommand(drive, arm, leds, nextAvailableTarget, false);
            }, Set.of(drive, arm))).unless(() -> grabbingCoralFailed) //
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
                var bluePose = AutoBuilder.shouldFlip() ? FlippingUtil.flipFieldPose(RobotState.getInstance().getPose()) : RobotState.getInstance().getPose();

                isLeft.value = bluePose.getY() > FieldConstants.fieldWidth / 2.;
                angle.value = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red ? Rotation2d.kZero : Rotation2d.k180deg;
                endPose.value = new Pose2d(RobotState.getInstance().getPose().getTranslation(), angle.value.rotateBy(Rotation2d.fromDegrees(180 + (isLeft.value ? -20 : 20))));
            }),
            // TODO: Mirror this rotation when on the other side of the field
            DriveCommands.driveStraightCommand(
                drive, moveSpeedMPS,
                () -> angle.value, () -> angle.value.rotateBy(Rotation2d.fromDegrees(180 + (isLeft.value ? 20 : -20)))
            ).until(() -> {
                if(intake.intakeSensorTriggered()) return true;

                // If the robot is at risk of running into the wall, stop.
                var robotPosition = RobotState.getInstance().getPose();
                var nextPosition = robotPosition.exp(drive.getChassisSpeeds().toTwist2d(0.75));
                if(nextPosition.getX() < 0.0 || nextPosition.getY() < 0.0
                    || nextPosition.getX() > FieldConstants.fieldLength
                    || nextPosition.getY() > FieldConstants.fieldWidth) {
                    return true;
                }
                
                return false;
            }).withTimeout(3.0),
            new DriveToPose(drive, () -> endPose.value, true).withTimeout(2.0)
        );
        // @formatter:on
    }
}
