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

    static {
        elevatorPitchDownHeightReduction.initDefault(5);
        pitchDownPitchReduction.initDefault(10);
        gamePieceEjectVelocity.initDefault(10);
        driveForwardTime.initDefault(0.5);
        branchScorePitch.initDefault(55.);
        L4ScoreHeightFromTop.initDefault(3);
        L3ScoreHeight.initDefault(26.5);
        L2ScoreHeight.initDefault(10.);
        L1ScoreHeight.initDefault(15.);
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
                return new ArmState(Rotation2d.fromDegrees(0), Inches.of(L1ScoreHeight.get()), WristRotation.Vertical,
                    EndEffectorState.hold());
        }

        return new ArmState(pitch, height, WristRotation.HorizontalFlipped, EndEffectorState.hold());
    }

    public static Command prepForScoring(ReefLevel level, Arm arm) {
        return arm.goToStateCommand(getStartingState(level));
    }

    public static Command scoreAtLevel(ReefLevel level, Arm arm, Drive drive) {
        if(level == ReefLevel.L1) return troughScoringSequence(arm);

        ArmState startState = getStartingState(level);
        ArmState pitchDownState = new ArmState(
            startState.pitch().minus(Rotation2d.fromDegrees(pitchDownPitchReduction.get())),
            startState.height().minus(Inches.of(elevatorPitchDownHeightReduction.get())),
            WristRotation.HorizontalFlipped, EndEffectorState.velocity(gamePieceEjectVelocity.get()));
        return Commands.sequence(
            Commands.parallel(arm.goToStateCommand(pitchDownState),
                DriveCommands.driveStraightCommand(drive, Units.feetToMeters(-2.0), 0.75)),
            arm.goToStateCommand(ArmConstants.restingState));
    }

    private static Command troughScoringSequence(Arm arm) {
        return Commands.sequence(arm.goToStateCommand(new ArmState(Rotation2d.fromDegrees(0),
            Inches.of(L1ScoreHeight.get()), WristRotation.Vertical, EndEffectorState.velocity(10.))));
    }
}
