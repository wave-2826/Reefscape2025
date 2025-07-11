package frc.robot.commands;

import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.commands.arm.ScoringSequenceCommands;
import frc.robot.commands.drive.ReefLineupCommand;
import frc.robot.commands.drive.ReefLineupCommand.FinishBehavior;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.commands.util.RestartWhenCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.LEDState;
import frc.robot.util.Container;
import frc.robot.util.DriverStationInterface;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ReefTarget;

public class AutoScoreCommands {
    private static final LoggedTunableNumber autoAlignTweakAmount = new LoggedTunableNumber(//
        "AutoScore/AutoAlignTweakInches", 4.5);
    private static final LoggedTunableNumber autoAlignTweakAmountL1 = new LoggedTunableNumber(//
        "AutoScore/AutoAlignTweakL1Inches", 7);

    private static final BooleanSupplier resetElevatorDuringScoring = () -> true; // DriverStation::isTeleop;

    public static Command autoAlign(Drive drive, LEDs leds, ReefTarget target, DoubleSupplier tweakX,
        DoubleSupplier tweakY, Supplier<FinishBehavior> finishBehavior, BooleanConsumer lineupFeedback) {
        double tweakAmount = Units
            .inchesToMeters(target.level() == ReefLevel.L1 ? autoAlignTweakAmountL1.get() : autoAlignTweakAmount.get());

        Supplier<Transform2d> getFieldRelativeOffset = () -> new Transform2d(
            new Translation2d(-tweakY.getAsDouble() * tweakAmount, -tweakX.getAsDouble() * tweakAmount),
            Rotation2d.kZero);

        return new ReefLineupCommand(drive, leds, target, getFieldRelativeOffset, finishBehavior, lineupFeedback);
    }

    private static class AutoScoreState {
        public ReefTarget target;

        public AutoScoreState(ReefTarget target) {
            this.target = target;
        }
    }

    private static Command resetArmCommand(Arm arm) {
        return arm.waitForCorrectAbsolute().onlyIf(resetElevatorDuringScoring);
    }

    /**
     * Gets a command that automatically scores at the selected level on the driver station interface. This is the
     * version of our automatic scoring mechanism intended for use in autonomous.
     * <p>
     * Unlike the teleop version, this doesn't drive backward after scoring.
     * @param drive The drive subsystem
     * @param arm The arm subsystem
     * @param leds The LEDs subsystem
     * @param target The target to score at
     * @param driveBackward If the robot should drive backward after scoring.
     * @return
     */
    public static Command autoScoreCommand(Drive drive, Arm arm, LEDs leds, ReefTarget target, boolean driveBackward) {
        return autoScoreCommand(drive, arm, leds, target, Optional.empty(), () -> false, () -> 0, () -> 0, null,
            driveBackward);
    }

    /**
     * Gets a command that automatically scores at the selected level on the driver station interface. This is the
     * version of our automatic scoring mechanism intended for use in autonomous.
     * @param drive The drive subsystem
     * @param arm The arm subsystem
     * @param leds The LEDs subsystem
     * @param target The target to score at
     * @param finishSequence If present, the reef lineup waits for this to be true before finishing. If not present, the
     *            reef lineup finishes when the target is fully aligned.
     * @param finishSequenceSlow If not null, this acts like finishSequence but finishes with a slow move downward. This
     *            is pretty hacky, and only works if finishSequence is present.
     * @param tweakX The amount to tweak the X position during reef lineup
     * @param tweakY The amount to tweak the Y position during reef lineup
     * @param lineupFeedback A function to call during reef lineup with whether the controller is at its target. Can be
     *            null.
     * @return
     */
    public static Command autoScoreCommand(Drive drive, Arm arm, LEDs leds, ReefTarget target,
        Optional<BooleanSupplier> finishSequence, BooleanSupplier finishSequenceSlow, DoubleSupplier tweakX,
        DoubleSupplier tweakY, BooleanConsumer lineupFeedback, boolean driveBackward) {
        Container<Boolean> finishOnceAtSetpoint = new Container<>(false);
        Optional<BooleanSupplier> finish = finishSequence
            .map(f -> (() -> (f.getAsBoolean() || finishSequenceSlow.getAsBoolean())));

        // @formatter:off
        return Commands.sequence(
            Commands.parallel(
                Commands.runOnce(() -> {
                    finishOnceAtSetpoint.value = false;
                    Arm.resetWristOverride();
                }),
                Commands.defer(() -> autoAlign(drive, leds, target, tweakX, tweakY, () -> {
                    if(finishSequence.isPresent()
                        && RobotState.getInstance().getPose().minus(ReefLineupCommand.getLineupPose(target))
                            .getTranslation().getNorm() > Units.inchesToMeters(6)
                        && finishSequence.get().getAsBoolean() && !finishOnceAtSetpoint.value) {
                        finishOnceAtSetpoint.value = true;
                    }

                    if(finishOnceAtSetpoint.value || finish.isEmpty()) {
                        return FinishBehavior.EndOnceAtSetpoint;
                    } else if(finish.orElse(() -> false).getAsBoolean()) {
                        return FinishBehavior.Finish;
                    }
                    return FinishBehavior.DoNotFinish;
                }, lineupFeedback).withTimeout(DriverStation.isAutonomous() ? 5. : 10000.), Set.of(drive)),

                Commands.sequence(
                    Commands.waitUntil(() -> !IntakeCommands.waitingForPiece).withTimeout(3).onlyIf(DriverStation::isAutonomous),
                    ScoringSequenceCommands.middleArmMovement(target.level(), arm).withTimeout(3),
                    resetArmCommand(arm)
                )
            ),
            Commands.either(
                ScoringSequenceCommands.scoreAtLevelSlowly(target.level(), arm)
                    .deadlineFor(leds.runStateCommand(LEDState.AutoScoring)),
                ScoringSequenceCommands
                    .scoreAtLevel(target.level(), arm, drive, target.branch().face.getFieldAngle(), !driveBackward)
                    .deadlineFor(leds.runStateCommand(LEDState.AutoScoring)),
                finishSequenceSlow::getAsBoolean
            ),

            Commands.waitUntil(() -> !finishSequence.get().getAsBoolean()).onlyIf(finishSequence::isPresent)
        ).withName("AutoScore" + target.toString());
        // @formatter:on
    }

    /**
     * Gets a command that automatically scores at the selected level on the driver station interface. This is the
     * version of our automatic scoring mechanism intended for use in teleop.
     * @param drive The drive subsystem
     * @param arm The arm subsystem
     * @param leds The LEDs subsystem
     * @param finishSequence The reef lineup waits for this to be true before finishing.
     * @param tweakX The amount to tweak the X position during reef lineup
     * @param tweakY The amount to tweak the Y position during reef lineup
     * @param lineupFeedback A function to call during reef lineup with whether the controller is at its target. Can be
     *            null.
     * @return
     */
    public static Command autoScoreTeleopCommand(Drive drive, Arm arm, LEDs leds, BooleanSupplier finishSequence,
        BooleanSupplier finishSequenceSlow, DoubleSupplier tweakX, DoubleSupplier tweakY,
        BooleanConsumer lineupFeedback) {
        AutoScoreState state = new AutoScoreState(DriverStationInterface.getInstance().getReefTarget());

        return new RestartWhenCommand(
            () -> autoScoreCommand(drive, arm, leds, state.target, Optional.of(finishSequence), finishSequenceSlow,
                tweakX, tweakY, lineupFeedback, true),

            // Restart when our target changes
            () -> {
                var newTarget = DriverStationInterface.getInstance().getReefTarget();
                if(!newTarget.equals(state.target)) {
                    state.target = newTarget;
                    return true;
                }
                return false;
            }, Set.of(drive, arm));
    }

    /**
     * Gets a command that automatically removes algae at the selected level on the driver station interface.
     * @param drive The drive subsystem
     * @param arm The arm subsystem
     * @param leds The LEDs subsystem
     * @param target The target to score at
     * @param tweakX The amount to tweak the X position during reef lineup
     * @param tweakY The amount to tweak the Y position during reef lineup
     * @return
     */
    public static Command removeAlgaeCommand(Drive drive, Arm arm, LEDs leds, ReefTarget target,
        BooleanSupplier finishSequence, DoubleSupplier tweakX, DoubleSupplier tweakY) {
        return Commands.sequence(//
            Commands.runOnce(Arm::resetWristOverride), //
            ScoringSequenceCommands.prepForAlgaeRemoval(target.level(), arm).withTimeout(2), resetArmCommand(arm),
            autoAlign(drive, leds, target, tweakX, tweakY,
                () -> finishSequence.getAsBoolean() ? FinishBehavior.Finish : FinishBehavior.DoNotFinish, null), //
            ScoringSequenceCommands.removeAlgae(target.level(), arm, drive, target.branch().face.getFieldAngle())
                .raceWith(leds.runStateCommand(LEDState.AutoScoring))//
        ).withName("RemoveAlgae" + target.toString());
    }

    /**
     * Gets a command that automatically removes algae at the selected level on the driver station interface.
     * @param drive The drive subsystem
     * @param arm The arm subsystem
     * @param leds The LEDs subsystem
     * @param finishSequence The reef lineup waits for this to be true before finishing.
     * @param tweakX The amount to tweak the X position during reef lineup
     * @param tweakY The amount to tweak the Y position during reef lineup
     * @return
     */
    public static Command removeAlgaeTeleopCommand(Drive drive, Arm arm, LEDs leds, BooleanSupplier finishSequence,
        DoubleSupplier tweakX, DoubleSupplier tweakY) {
        AutoScoreState state = new AutoScoreState(DriverStationInterface.getInstance().getReefTarget());

        return new RestartWhenCommand(
            () -> removeAlgaeCommand(drive, arm, leds, state.target.asAlgae(), finishSequence, tweakX, tweakY),

            // Restart when our target changes
            () -> {
                var newTarget = DriverStationInterface.getInstance().getReefTarget();
                if(!newTarget.equals(state.target)) {
                    state.target = newTarget;
                    return true;
                }
                return false;
            }, Set.of(drive, arm));
    }
}
