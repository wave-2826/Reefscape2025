package frc.robot.commands.arm;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
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
import frc.robot.util.LoggedTunableNumber;

public class ScoringSequenceCommands {
    private static LoggedTunableNumber elevatorPitchDownHeightReduction = new LoggedTunableNumber(
        "AutoScore/PitchDownHeightReduction");
    private static LoggedTunableNumber pitchDownPitchReduction = new LoggedTunableNumber(
        "AutoScore/PitchDownPitchReduction");
    private static LoggedTunableNumber gamePieceEjectVelocity = new LoggedTunableNumber(
        "AutoScore/GamePieceEjectVelocity");
    private static LoggedTunableNumber driveForwardTime = new LoggedTunableNumber("AutoScore/DriveForwardTime");
    private static LoggedTunableNumber branchScorePitch = new LoggedTunableNumber("AutoScore/BranchScorePitch");
    private static LoggedTunableNumber L4ScoreHeightFromTop = new LoggedTunableNumber("AutoScore/L4ScoreHeightFromTop");
    private static LoggedTunableNumber L3ScoreHeight = new LoggedTunableNumber("AutoScore/L3ScoreHeight");
    private static LoggedTunableNumber L2ScoreHeight = new LoggedTunableNumber("AutoScore/L2ScoreHeight");
    private static LoggedTunableNumber L1ScoreHeight = new LoggedTunableNumber("AutoScore/L1ScoreHeight");
    private static LoggedTunableNumber preScoreElevatorHeight = new LoggedTunableNumber(
        "AutoScore/PreScoreElevatorHeight");
    private static LoggedTunableNumber prepPitch = new LoggedTunableNumber("AutoScore/PrepPitch");

    static {
        elevatorPitchDownHeightReduction.initDefault(5);
        pitchDownPitchReduction.initDefault(10);
        gamePieceEjectVelocity.initDefault(2);
        driveForwardTime.initDefault(0.5);
        branchScorePitch.initDefault(55.);
        L4ScoreHeightFromTop.initDefault(6);
        L3ScoreHeight.initDefault(26.5);
        L2ScoreHeight.initDefault(10.);
        L1ScoreHeight.initDefault(15.);
        preScoreElevatorHeight.initDefault(12.5);
        prepPitch.initDefault(80);
    }

    private static ArmState getL1StartingState() {
        return new ArmState(Rotation2d.fromDegrees(0), Inches.of(L1ScoreHeight.get()), WristRotation.Vertical,
            EndEffectorState.hold());
    }

    public static ArmState getStartingState(ReefLevel level) {
        Rotation2d pitch = Rotation2d.fromDegrees(branchScorePitch.get());
        Distance height;
        switch(level) {
            case L4:
                height = ArmConstants.ElevatorConstants.maxElevatorHeight.minus(Inches.of(L4ScoreHeightFromTop.get()));
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

        return new ArmState(pitch, height, WristRotation.HorizontalFlipped, EndEffectorState.hold());
    }

    public static Command middleArmMovement(ReefLevel level, Arm arm) {
        ArmState startingState = getStartingState(level);
        return Commands.sequence( //
            arm.goToStateCommand(startingState.withPitch(Rotation2d.fromDegrees(prepPitch.get()))),
            arm.goToStateCommand(startingState));
    }

    public static Command prepForScoring(ReefLevel level, Arm arm) {
        if(level == ReefLevel.L1) { return arm.goToStateCommand(getL1StartingState()); }
        return arm.goToStateCommand(new ArmState(Rotation2d.fromDegrees(prepPitch.get()),
            Inches.of(preScoreElevatorHeight.get()), WristRotation.HorizontalFlipped, EndEffectorState.hold()));
    }

    public static Command scoreAtLevel(ReefLevel level, Arm arm, Drive drive) {
        if(level == ReefLevel.L1) return troughScoringSequence(arm);

        ArmState startState = getStartingState(level);
        ArmState pitchDownState = new ArmState(
            startState.pitch().minus(Rotation2d.fromDegrees(pitchDownPitchReduction.get())),
            startState.height().minus(Inches.of(elevatorPitchDownHeightReduction.get())),
            WristRotation.HorizontalFlipped, EndEffectorState.velocity(gamePieceEjectVelocity.get()));
        return Commands.sequence(arm.goToStateCommand(startState),
            Commands.parallel(arm.goToStateCommand(pitchDownState),
                DriveCommands.driveStraightCommand(drive, Units.feetToMeters(-2.0), 0.5)),
            arm.goToStateCommand(ArmConstants.restingState)).finallyDo(() -> {
                arm.goToStateCommand(ArmConstants.restingState).schedule();
            });
    }

    private static Command troughScoringSequence(Arm arm) {
        return Commands.sequence(arm.goToStateCommand(new ArmState(Rotation2d.fromDegrees(0),
            Inches.of(L1ScoreHeight.get()), WristRotation.Vertical, EndEffectorState.velocity(10.))));
    }
}
