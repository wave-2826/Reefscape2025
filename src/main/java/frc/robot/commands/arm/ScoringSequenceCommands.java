package frc.robot.commands.arm;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.EndEffectorState;
import frc.robot.subsystems.arm.ArmState.WristRotation;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;

/**
 * This is a disaster... hopefully I rarely ever have to touch this.
 */
public class ScoringSequenceCommands {
    private static LoggedTunableNumber elevatorScoreHeightReduction = new LoggedTunableNumber(//
        "AutoScore/ScoreHeightReduction", 6);
    private static LoggedTunableNumber gamePieceEjectVelocity = new LoggedTunableNumber(//
        "AutoScore/GamePieceEjectVelocity", 17);
    private static LoggedTunableNumber branchScorePitch = new LoggedTunableNumber(//
        "AutoScore/BranchScorePitch", 65.);
    private static LoggedTunableNumber L4ScorePitch = new LoggedTunableNumber(//
        "AutoScore/L4ScorePitch", 55.);
    private static LoggedTunableNumber L1ScoreHeight = new LoggedTunableNumber(//
        "AutoScore/L1ScoreHeight", 14);
    private static LoggedTunableNumber L2ScoreHeight = new LoggedTunableNumber(//
        "AutoScore/L2ScoreHeight", 10);
    private static LoggedTunableNumber L3ScoreHeight = new LoggedTunableNumber(//
        "AutoScore/L3ScoreHeight", 26);
    private static LoggedTunableNumber L4ScoreHeightFromTop = new LoggedTunableNumber(//
        "AutoScore/L4ScoreHeightFromTop", 10);
    private static LoggedTunableNumber preScoreElevatorHeight = new LoggedTunableNumber(//
        "AutoScore/PreScoreElevatorHeight", 15.5);
    private static LoggedTunableNumber branchScorePitchDown = new LoggedTunableNumber(//
        "AutoScore/BranchScorePitchDown", 40);

    /**
     * Gets the starting state for scoring at level 1.
     * @return
     */
    private static ArmState getL1StartingState() {
        return new ArmState(Rotation2d.fromDegrees(0), Inches.of(L1ScoreHeight.get()), WristRotation.Vertical,
            EndEffectorState.hold());
    }

    /**
     * Gets the starting state for scoring at a given level.
     * @param level
     * @return
     */
    public static ArmState getStartingState(ReefLevel level) {
        Rotation2d pitch = Rotation2d.fromDegrees(branchScorePitch.get());
        Distance height;
        boolean flipped = true;
        switch(level) {
            case L4:
                height = ArmConstants.ElevatorConstants.maxElevatorHeight.minus(Inches.of(L4ScoreHeightFromTop.get()));
                pitch = Rotation2d.fromDegrees(L4ScorePitch.get());
                flipped = true;
                break;
            case L3:
                height = Inches.of(L3ScoreHeight.get());
                break;
            case L2:
                height = Inches.of(L2ScoreHeight.get());
                break;
            default:
                return getL1StartingState();
        }

        return new ArmState(pitch, height, flipped ? WristRotation.HorizontalFlipped : WristRotation.Horizontal,
            EndEffectorState.hold());
    }

    /**
     * Goes to an arm state that prepares to score by holding the piece upward. Should be run during coarse alignment.
     * @return
     */
    public static Command prepForScoring(ReefLevel level, Arm arm) {
        if(level == ReefLevel.L1) { return arm.goToStateCommand(getL1StartingState()); }

        ArmState startingState = getStartingState(level);
        return arm.goToStateCommand(new ArmState(Rotation2d.fromDegrees(80), Inches.of(preScoreElevatorHeight.get()),
            startingState.wristRotation(), EndEffectorState.hold()));
    }

    /**
     * The middle movement as part of the scoring sequence. Should be run during close alignment after prepForScoring.
     * This is the "main" arm position as part of the scoring sequence; the arm will be lined up with the branch post.
     * @return
     */
    public static Command middleArmMovement(ReefLevel level, Arm arm) {
        if(level == ReefLevel.L1) { return arm.goToStateCommand(getL1StartingState()); }

        ArmState startingState = getStartingState(level);
        return arm.goToStateCommand(startingState).withTimeout(0.75);
    }

    /**
     * A full scoring sequence that scores at the given level. The arm should have been prepped for scoring and executed
     * the middle arm movement before this command is run.
     * @param level
     * @param arm
     * @param drive
     * @return
     */
    public static Command scoreAtLevel(ReefLevel level, Arm arm, Drive drive, Rotation2d fieldAngle) {
        if(level == ReefLevel.L1) return troughScoringSequence(arm);

        if(level == ReefLevel.L4) {
            ArmState startState = getStartingState(level);
            ArmState scoreDownState = new ArmState(startState.pitch().minus(Rotation2d.fromDegrees(40)),
                startState.height().minus(Inches.of(elevatorScoreHeightReduction.get())), startState.wristRotation(),
                EndEffectorState.velocity(gamePieceEjectVelocity.get()));

            // @formatter:off
            return Commands.sequence(
                Commands.parallel(
                    arm.goToStateCommand(scoreDownState).withTimeout(0.75),
                    DriveCommands.driveStraightCommand(drive, Units.feetToMeters(-2.5), 0.75, fieldAngle)
                ),
                arm.goToStateCommand(ArmConstants.restingState)
            );
            // @formatter:on
        }

        ArmState startState = getStartingState(level);
        ArmState scoreDownState = new ArmState(startState.pitch(),
            startState.height().minus(Inches.of(elevatorScoreHeightReduction.get())), startState.wristRotation(),
            Constants.isSim ? EndEffectorState.velocity(gamePieceEjectVelocity.get()) : EndEffectorState.hold());
        ArmState scoreDownState2 = new ArmState(
            startState.pitch().minus(Rotation2d.fromDegrees(branchScorePitchDown.get())),
            startState.height().minus(Inches.of(elevatorScoreHeightReduction.get())), startState.wristRotation(),
            EndEffectorState.velocity(gamePieceEjectVelocity.get()));

        // @formatter:off
        return Commands.sequence(
            Commands.parallel(
                Commands.sequence(
                    Commands.waitSeconds(0.2),
                    arm.goToStateCommand(scoreDownState).withTimeout(0.75)
                ),
                DriveCommands.driveStraightCommand(drive, Units.feetToMeters(2.5), 0.4, fieldAngle)
            ).withTimeout(0.75),
            Commands.parallel(
                arm.goToStateCommand(scoreDownState2).withTimeout(0.75),
                DriveCommands.driveStraightCommand(drive, Units.feetToMeters(-1.5), 0.4, fieldAngle)
            ),
            arm.goToStateCommand(ArmConstants.restingState)
        ).withName("ScoreAt" + level.name() + "Sequence");
        // @formatter:on
    }

    /**
     * The trough scoring sequence.
     * @param arm
     * @return
     */
    private static Command troughScoringSequence(Arm arm) {
        return Commands.sequence(arm.goToStateCommand(new ArmState(Rotation2d.fromDegrees(0),
            Inches.of(L1ScoreHeight.get()), WristRotation.Vertical, EndEffectorState.velocity(10.))))
            .withName("TroughScoringSequence");
    }
}
