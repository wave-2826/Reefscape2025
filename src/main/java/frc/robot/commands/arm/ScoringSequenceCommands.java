package frc.robot.commands.arm;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
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

/**
 * This is a disaster... hopefully I rarely ever have to touch this.
 */
public class ScoringSequenceCommands {
    private static LoggedTunableNumber elevatorScoreHeightReduction = new LoggedTunableNumber(//
        "AutoScore/ScoreHeightReduction", 4);
    private static LoggedTunableNumber gamePieceEjectVelocity = new LoggedTunableNumber(//
        "AutoScore/GamePieceEjectVelocity", 6);
    private static LoggedTunableNumber branchScorePitch = new LoggedTunableNumber(//
        "AutoScore/BranchScorePitch", 58);
    private static LoggedTunableNumber branchScorePitchDown = new LoggedTunableNumber(//
        "AutoScore/BranchScorePitchDown", 50);
    private static LoggedTunableNumber branchScorePitchDownL2 = new LoggedTunableNumber(//
        "AutoScore/BranchScorePitchDownL2", 25);
    private static LoggedTunableNumber L4ScorePitch = new LoggedTunableNumber(//
        "AutoScore/L4ScorePitch", 40);
    private static LoggedTunableNumber L4PitchDown = new LoggedTunableNumber(//
        "AutoScore/L4PitchDown", 49);

    private static LoggedTunableNumber L1ScorePitch = new LoggedTunableNumber(//
        "AutoScore/L1ScorePitch", -50);
    private static LoggedTunableNumber L1BumpHeight = new LoggedTunableNumber(//
        "AutoScore/L1BumpHeight", 2);
    private static LoggedTunableNumber L1EjectSpeed = new LoggedTunableNumber(//
        "AutoScore/L1EjectSpeed", 7);

    private static LoggedTunableNumber[] levelScoreHeights = new LoggedTunableNumber[] {
        new LoggedTunableNumber("AutoScore/L1ScoreHeight", 22.5), //
        new LoggedTunableNumber("AutoScore/L2ScoreHeight", 5.5), //
        new LoggedTunableNumber("AutoScore/L3ScoreHeight", 21.5), //
        new LoggedTunableNumber("AutoScore/L4ScoreHeight", 50.75)
    };

    /**
     * Gets the starting state for scoring at a given level.
     * @param level
     * @return
     */
    public static ArmState getStartingState(ReefLevel level) {
        if(level == ReefLevel.L1) return getL1StartingState();

        Rotation2d pitch = Rotation2d.fromDegrees(branchScorePitch.get());
        Distance height = Inches.of(levelScoreHeights[level.ordinal()].get());
        boolean flipped = true;

        if(level == ReefLevel.L4) {
            pitch = Rotation2d.fromDegrees(L4ScorePitch.get());
        }

        WristRotation rotation = flipped ? WristRotation.HorizontalFlipped : WristRotation.Horizontal;
        return new ArmState(pitch, height, rotation, EndEffectorState.hold());
    }

    /**
     * The middle movement as part of the scoring sequence. Should be run during close alignment. This is the "main" arm
     * position as part of the scoring sequence; the arm will be lined up with the branch post.
     * @return
     */
    public static Command middleArmMovement(ReefLevel level, Arm arm) {
        return arm.goToStateCommand(() -> getStartingState(level)).withTimeout(0.75);
    }

    /**
     * A full scoring sequence that scores at the given level. The arm should have been prepped for scoring and executed
     * the middle arm movement before this command is run.
     * @param level
     * @param arm
     * @param drive If null, no "drive back" is performed.
     * @return
     */
    public static Command scoreAtLevel(ReefLevel level, Arm arm, Drive drive, Rotation2d fieldAngle,
        boolean minimalBackUp) {
        if(level == ReefLevel.L1) return troughScoringSequence(drive, arm, fieldAngle);

        if(level == ReefLevel.L4) {
            ArmState startState = getStartingState(level);
            ArmState scoreDownState = new ArmState(
                startState.pitch().minus(Rotation2d.fromDegrees(branchScorePitchDown.get())),
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

        ArmState startState = getStartingState(level);
        ArmState scoreDownState = new ArmState(
            startState.pitch()
                .minus(Rotation2d
                    .fromDegrees(level == ReefLevel.L2 ? branchScorePitchDownL2.get() : branchScorePitchDown.get())),
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
    public static Command scoreAtLevelSlowly(ReefLevel level, Arm arm) {
        if(level == ReefLevel.L1) return slowTroughScoringSequence(arm);

        Container<Double> startTime = new Container<Double>(0.);
        double scoreTime = 2.0;

        if(level == ReefLevel.L4) {
            ArmState startState = getStartingState(level);
            ArmState scoreDownState = new ArmState(startState.pitch().minus(Rotation2d.fromDegrees(L4PitchDown.get())),
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

        ArmState startState = getStartingState(level);
        ArmState scoreDownState = new ArmState(
            startState.pitch().minus(Rotation2d.fromDegrees(branchScorePitchDown.get())),
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
    private static ArmState getL1StartingState() {
        return new ArmState(//
            Rotation2d.fromDegrees(L1ScorePitch.get()), //
            Inches.of(levelScoreHeights[0].get() + L1BumpHeight.get()), //
            WristRotation.HorizontalFlipped, //
            EndEffectorState.hold() //
        );
    }

    /**
     * The trough scoring sequence.
     * @param arm
     * @return
     */
    private static Command troughScoringSequence(Drive drive, Arm arm, Rotation2d fieldAngle) {
        ArmState initialState = getL1StartingState();
        ArmState scoreState = initialState.withEndEffector(EndEffectorState.velocity(L1EjectSpeed.get()))
            .withHeight(initialState.height().minus(Inches.of(L1BumpHeight.get())));

        // @formatter:off
        return Commands.sequence(
            arm.goToStateCommand(initialState),
            arm.goToStateCommand(scoreState),
            Commands.waitSeconds(0.15),
            DriveCommands.driveStraightCommand(drive, Units.feetToMeters(-4), 1.0, () -> fieldAngle, null)
        ).withName("TroughScoringSequence");
        // @formatter:on
    }

    private static Command slowTroughScoringSequence(Arm arm) {
        ArmState initialState = getL1StartingState();
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
