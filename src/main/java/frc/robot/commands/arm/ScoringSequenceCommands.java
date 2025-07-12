package frc.robot.commands.arm;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.EndEffectorState;
import frc.robot.subsystems.arm.ArmState.WristRotation;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Container;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ReefTarget;

/**
 * This is a disaster... hopefully I rarely ever have to touch this.
 */
public class ScoringSequenceCommands {
    private static LoggedTunableNumber elevatorScoreHeightReduction = new LoggedTunableNumber(//
        "AutoScore/ScoreHeightReduction", 4);
    private static LoggedTunableNumber gamePieceEjectVelocity = new LoggedTunableNumber(//
        "AutoScore/GamePieceEjectVelocity", 6);
    private static LoggedTunableNumber[] branchScorePitches = new LoggedTunableNumber[] {
        new LoggedTunableNumber("AutoScore/L1ScorePitch", -50), //
        new LoggedTunableNumber("AutoScore/L2ScorePitch", 58), //
        new LoggedTunableNumber("AutoScore/L3ScorePitch", 58), //
        new LoggedTunableNumber("AutoScore/L4ScorePitch", 40)
    };
    private static LoggedTunableNumber[] branchScorePitchDown = new LoggedTunableNumber[] {
        null, //
        new LoggedTunableNumber("AutoScore/BranchScorePitchDownL2", 25), //
        new LoggedTunableNumber("AutoScore/BranchScorePitchDownL3", 35), //
        new LoggedTunableNumber("AutoScore/BranchScorePitchDownL4", 49),
    };

    private static LoggedTunableNumber L1BumpHeight = new LoggedTunableNumber(//
        "AutoScore/L1BumpHeight", 1);
    private static LoggedTunableNumber L1EjectSpeed = new LoggedTunableNumber(//
        "AutoScore/L1EjectSpeed", 7);

    private static LoggedTunableNumber[] levelScoreHeights = new LoggedTunableNumber[] {
        new LoggedTunableNumber("AutoScore/L1ScoreHeight", 23.5), //
        new LoggedTunableNumber("AutoScore/L2ScoreHeight", 5.5), //
        new LoggedTunableNumber("AutoScore/L3ScoreHeight", 21.5), //
        new LoggedTunableNumber("AutoScore/L4ScoreHeight", 50.75)
    };

    /**
     * Gets the starting state for scoring at a given level.
     * @param level
     * @return
     */
    public static ArmState getStartingState(ReefTarget target) {
        ReefLevel level = target.level();
        if(level == ReefLevel.L1) return getL1StartingState(target.branch().isLeft);

        Rotation2d pitch = Rotation2d.fromDegrees(//
            branchScorePitches[level.ordinal()].get() + (DriverStation.isAutonomous() ? 5 : 0));
        Distance height = Inches.of(levelScoreHeights[level.ordinal()].get());
        boolean flipped = true;

        WristRotation rotation = flipped ? WristRotation.HorizontalFlipped : WristRotation.Horizontal;
        return new ArmState(pitch, height, rotation, EndEffectorState.hold());
    }

    /**
     * The middle movement as part of the scoring sequence. Should be run during close alignment. This is the "main" arm
     * position as part of the scoring sequence; the arm will be lined up with the branch post.
     * @return
     */
    public static Command middleArmMovement(ReefTarget target, Arm arm) {
        return arm.goToStateCommand(() -> getStartingState(target)).withTimeout(0.75);
    }

    /**
     * A full scoring sequence that scores at the given level. The arm should have been prepped for scoring and executed
     * the middle arm movement before this command is run.
     * @param level
     * @param arm
     * @param drive If null, no "drive back" is performed.
     * @return
     */
    public static Command scoreAtLevel(ReefTarget target, Arm arm, Drive drive, Rotation2d fieldAngle,
        boolean minimalBackUp) {
        ReefLevel level = target.level();
        if(level == ReefLevel.L1) return troughScoringSequence(drive, arm, fieldAngle, target.branch().isLeft);

        if(level == ReefLevel.L4) {
            ArmState startState = getStartingState(target);
            ArmState scoreDownState = new ArmState(
                startState.pitch().minus(Rotation2d.fromDegrees(branchScorePitchDown[level.ordinal()].get())),
                startState.height().minus(Inches.of(elevatorScoreHeightReduction.get())), startState.wristRotation(),
                EndEffectorState.velocity(gamePieceEjectVelocity.get()));

            // @formatter:off
            return minimalBackUp ? arm.goToStateCommand(scoreDownState, 0.25).deadlineFor(
                Commands.waitSeconds(0.1)
                    .andThen(DriveCommands.driveStraightCommand(drive, Units.feetToMeters(-8), () -> fieldAngle, null))
            ) : Commands.sequence(
                Commands.parallel(
                    arm.goToStateCommand(scoreDownState, 0.25),
                    DriveCommands.driveStraightCommand(drive, Units.feetToMeters(-1.5), 1.5, () -> fieldAngle, null)
                ),
                arm.goToStateCommand(ArmConstants.restingState)
            );
            // @formatter:on
        }

        ArmState startState = getStartingState(target);
        ArmState scoreDownState = new ArmState(
            startState.pitch().minus(Rotation2d.fromDegrees(branchScorePitchDown[level.ordinal()].get())),
            startState.height().minus(Inches.of(elevatorScoreHeightReduction.get())), startState.wristRotation(),
            EndEffectorState.velocity(gamePieceEjectVelocity.get()));

        // @formatter:off
        return Commands.parallel(
            Commands.runOnce(Arm::resetWristOverride),
            arm.goToStateCommand(scoreDownState).withTimeout(0.75),
            Commands.sequence(
                Commands.waitSeconds(0.15),
                DriveCommands.driveStraightCommand(drive, Units.feetToMeters(minimalBackUp ? -4 : -3), minimalBackUp ? 0.3 : 1.5, () -> fieldAngle, null)
            )
        ).withName("ScoreAt" + level.name() + "Sequence");
        // @formatter:on
    }

    // HACK aaahahh
    public static Command scoreAtLevelSlowly(ReefTarget target, Arm arm) {
        ReefLevel level = target.level();
        if(level == ReefLevel.L1) return slowTroughScoringSequence(arm, target.branch().isLeft);

        Container<Double> startTime = new Container<Double>(0.);
        double scoreTime = 2.0;

        if(level == ReefLevel.L4) {
            ArmState startState = getStartingState(target);
            ArmState scoreDownState = new ArmState(
                startState.pitch().minus(Rotation2d.fromDegrees(branchScorePitchDown[3].get())),
                startState.height().minus(Inches.of(elevatorScoreHeightReduction.get())), startState.wristRotation(),
                EndEffectorState.velocity(1.));

            // @formatter:off
            return Commands.sequence(
                Commands.runOnce(() -> startTime.value = Timer.getTimestamp()),
                arm.setTargetStateCommand(() -> startState.lerp(scoreDownState, (Timer.getTimestamp() - startTime.value) / scoreTime))
                    .withTimeout(scoreTime)
            );
            // @formatter:on
        }

        ArmState startState = getStartingState(target);
        ArmState scoreDownState = new ArmState(
            startState.pitch().minus(Rotation2d.fromDegrees(branchScorePitchDown[level.ordinal()].get())),
            startState.height().minus(Inches.of(elevatorScoreHeightReduction.get())), startState.wristRotation(),
            EndEffectorState.velocity(1.));

        // @formatter:off
        return Commands.sequence(
            Commands.runOnce(Arm::resetWristOverride),
            Commands.runOnce(() -> startTime.value = Timer.getTimestamp()),
            arm.setTargetStateCommand(() -> startState.lerp(scoreDownState, (Timer.getTimestamp() - startTime.value) / scoreTime)).withTimeout(scoreTime)
        ).withName("ScoreAt" + level.name() + "SlowlySequence");
        // @formatter:on
    }

    /**
     * Goes to an arm state that prepares to remove algae.
     * @return
     */
    public static Command prepForAlgaeRemoval(ReefLevel level, Arm arm) {
        if(level == ReefLevel.L3) {
            return arm.goToStateCommand(new ArmState(Rotation2d.fromDegrees(0), Meters.of(0.62), WristRotation.Vertical,
                EndEffectorState.hold()));
        } else {
            return arm.goToStateCommand(new ArmState(Rotation2d.fromDegrees(0), Meters.of(0.98), WristRotation.Vertical,
                EndEffectorState.hold()));
        }
    }

    /**
     * A sequence to remove algae at the given level.
     * @param level
     * @param arm
     * @param drive
     * @return
     */
    public static Command removeAlgae(ReefLevel level, Arm arm, Drive drive, Rotation2d fieldAngle) {
        if(level == ReefLevel.L3) {
            return Commands.parallel(
                arm.goToStateCommand(new ArmState(Rotation2d.fromDegrees(30), Inches.of(25), WristRotation.Vertical,
                    EndEffectorState.velocity(12))),
                DriveCommands.driveStraightCommand(drive, Units.feetToMeters(-2.5), 1.5, () -> fieldAngle, null));
        } else {
            return Commands.parallel(
                arm.goToStateCommand(new ArmState(Rotation2d.fromDegrees(-30), Inches.of(37), WristRotation.Vertical,
                    EndEffectorState.velocity(12))),
                DriveCommands.driveStraightCommand(drive, Units.feetToMeters(-2.5), 1.5, () -> fieldAngle, null));
        }
    }

    /**
     * Gets the starting state for scoring at level 1.
     * @return
     */
    private static ArmState getL1StartingState(boolean isLeft) {
        return new ArmState(//
            Rotation2d.fromDegrees(branchScorePitches[0].get()), //
            Inches.of(levelScoreHeights[0].get() + L1BumpHeight.get()), //
            isLeft ? WristRotation.VerticalFlipped : WristRotation.Vertical, //
            EndEffectorState.hold() //
        );
    }

    /**
     * The trough scoring sequence.
     * @param arm
     * @return
     */
    private static Command troughScoringSequence(Drive drive, Arm arm, Rotation2d fieldAngle, boolean isLeft) {
        ArmState initialState = getL1StartingState(isLeft);
        ArmState scoreState = initialState.withEndEffector(EndEffectorState.velocity(L1EjectSpeed.get()))
            .withHeight(initialState.height().minus(Inches.of(L1BumpHeight.get())));

        // @formatter:off
        return Commands.sequence(
            arm.goToStateCommand(scoreState, 0.5),
            DriveCommands.driveStraightCommand(drive, Units.feetToMeters(-4), 1.0, () -> fieldAngle, null)
        ).withName("TroughScoringSequence");
        // @formatter:on
    }

    private static Command slowTroughScoringSequence(Arm arm, boolean isLeft) {
        ArmState initialState = getL1StartingState(isLeft);
        ArmState scoreState = initialState.withEndEffector(EndEffectorState.velocity(L1EjectSpeed.get()))
            .withHeight(initialState.height().minus(Inches.of(L1BumpHeight.get())));

        // @formatter:off
        return Commands.sequence(
            arm.goToStateCommand(initialState),
            arm.goToStateCommand(scoreState)
        ).withName("TroughScoringSequence");
        // @formatter:on
    }
}
