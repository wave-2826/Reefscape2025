package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefFace;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.commands.arm.ScoringSequenceCommands;
import frc.robot.commands.drive.CloseLineupCommand;
import frc.robot.commands.util.RestartWhenCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.LEDState;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.DriverStationInterface;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ReefTarget;

public class AutoScoreCommands {
    /**
     * The distance from the reef branch to the center of the robot when lining up to score L1 in inches.
     */
    private static final LoggedTunableNumber robotReefLineupL1Distance = new LoggedTunableNumber(//
        "AutoScore/L1ReefLineupDistance", 33.5);

    /**
     * The distance from the reef branch to the center of the robot when lining up to score L2 in inches.
     */
    private static final LoggedTunableNumber robotReefLineupL2Distance = new LoggedTunableNumber(//
        "AutoScore/L2ReefLineupDistance", 20.5);

    /**
     * The distance from the reef branch to the center of the robot when lining up to score L3 in inches.
     */
    private static final LoggedTunableNumber robotReefLineupL3Distance = new LoggedTunableNumber(//
        "AutoScore/L3ReefLineupDistance", 21.75);

    /**
     * The distance from the reef branch to the center of the robot when lining up to score L4 in inches.
     */
    private static final LoggedTunableNumber robotReefLineupL4Distance = new LoggedTunableNumber(//
        "AutoScore/L4ReefLineupDistance", 24.25);

    /**
     * The amount the driver can tweak the auto lineup position, in inches.
     */
    private static final LoggedTunableNumber autoAlignTweakAmount = new LoggedTunableNumber(//
        "AutoScore/AutoAlignTweakInches", 4.5);

    /**
     * A tweaking factor added to our horizontal lineup distance from the center between two branches.
     */
    private static final LoggedTunableNumber centerDistanceTweak = new LoggedTunableNumber(//
        "AutoScore/CenterDistanceTweak", 0.5);

    /**
     * Gets a command that pathfinds to the target pose and precisely aligns to it. Because PathPlanner's default
     * pathfinding command is intended for long distances and doesn't move the robot if it's currently in the target
     * grid cell, we implement a second phase to align using a PID controller. Calls the passed commands during
     * different lineup stages.
     * <p>
     * This takes... a lot of parameters. Maybe we can clean it up a bit, but it needs to be adaptable between teleop
     * and autonomous.
     * @param drive The drive subsystem
     * @param vision The vision subsystem
     * @param target The reef target to align to
     * @param distanceAwayInches The distance away in inches.
     * @param alignCenter If the robot should align to the center instead of branch side.
     * @param coarseLineupCommand The command to run during coarse lineup
     * @param closeLineupCommand The command to run during close lineup
     * @param tweakX The amount to tweak the X position during close lineup
     * @param tweakY The amount to tweak the Y position during close lineup
     * @param finishSequence If present, the close lineup waits for this to be true before finishing. If not present,
     *            the close lineup finishes when the target is fully aligned.
     * @param lineupFeedback A function to call during close lineup with whether the controller is at its target. Can be
     *            null.
     * @return
     */
    public static Command autoAlignSequence(Drive drive, Vision vision, LEDs leds, ReefTarget target,
        double distanceAwayInches, boolean alignCenter, Command coarseLineupCommand, Command closeLineupCommand,
        DoubleSupplier tweakX, DoubleSupplier tweakY, Optional<BooleanSupplier> finishSequence,
        BooleanConsumer lineupFeedback) {

        boolean isLeft = target.branch().isLeft;

        double centerDistance = FieldConstants.reefBranchSeparation.in(Meters) / 2.
            + Units.inchesToMeters(centerDistanceTweak.get());
        Transform2d tagRelativeOffset = new Transform2d(new Translation2d(Units.inchesToMeters(distanceAwayInches),
            alignCenter ? 0. : (isLeft ? -centerDistance : centerDistance)), Rotation2d.k180deg);

        ReefFace reefFace = target.branch().face;

        Pose2d initialLineupPosition = reefFace.blueTagPose.transformBy(tagRelativeOffset)
            .plus(new Transform2d(new Translation2d(Units.inchesToMeters(-5.), 0.), Rotation2d.kZero));

        Supplier<Transform2d> getFieldRelativeOffset = () -> new Transform2d(
            new Translation2d(-tweakY.getAsDouble() * Units.inchesToMeters(autoAlignTweakAmount.get()),
                -tweakX.getAsDouble() * Units.inchesToMeters(autoAlignTweakAmount.get())),
            Rotation2d.kZero);

        // @formatter:off
        return Commands.sequence(
            Commands.runOnce(() -> {
                Logger.recordOutput("AutoScore/InitialLineupPose", initialLineupPosition);
            }), 

            Commands.runOnce(() -> Logger.recordOutput("AutoScore/RunningCloseLineup", true)),
            Commands.parallel(
                coarseLineupCommand.withTimeout(2),
                drive.runOnce(drive::stopWithX)
            ),
            closeLineupCommand.withDeadline(
                new CloseLineupCommand(
                    drive, vision, leds,
                    reefFace.getAprilTagID(),
                    tagRelativeOffset, getFieldRelativeOffset,
                    finishSequence, lineupFeedback
                ).withTimeout(DriverStation.isAutonomous() ? 4.0 : 10000).until(() -> {
                    // Last ditch effort at the end of auto
                    return DriverStation.isAutonomous() && DriverStation.getMatchTime() < 1.;
                }) // Final adjustment
            ) // Run during final adjustment
        ).finallyDo(() -> {
            Logger.recordOutput("AutoScore/RunningCloseLineup", false);
            Logger.recordOutput("AutoScore/InitialLineupPose", Pose2d.kZero);
        }).withName("AutoAlign" + target.toString()); // @formatter:on
    }

    private static class AutoScoreState {
        public ReefTarget target;

        public AutoScoreState(ReefTarget target) {
            this.target = target;
        }
    }

    /**
     * Gets a command that automatically scores at the selected level on the driver station interface. This is the
     * version of our automatic scoring mechanism intended for use in autonomous.
     * <p>
     * Unlike the teleop version, this doesn't drive backward after scoring.
     * @param drive The drive subsystem
     * @param vision The vision subsystem
     * @param arm The arm subsystem
     * @param leds The LEDs subsystem
     * @param target The target to score at
     * @param driveBackward If the robot should drive backward after scoring.
     * @return
     */
    public static Command autoScoreCommand(Drive drive, Vision vision, Arm arm, LEDs leds, ReefTarget target,
        boolean driveBackward) {
        return autoScoreCommand(drive, vision, arm, leds, target, Optional.empty(), () -> false, () -> 0, () -> 0, null,
            () -> true, driveBackward);
    }

    /**
     * Gets a command that automatically scores at the selected level on the driver station interface. This is the
     * version of our automatic scoring mechanism intended for use in autonomous.
     * @param drive The drive subsystem
     * @param vision The vision subsystem
     * @param arm The arm subsystem
     * @param leds The LEDs subsystem
     * @param target The target to score at
     * @param finishSequence If present, the close lineup waits for this to be true before finishing. If not present,
     *            the close lineup finishes when the target is fully aligned.
     * @param finishSequenceSlow If not null, this acts like finishSequence but finishes with a slow move downward. This
     *            is pretty hacky, and only works if finishSequence is present.
     * @param tweakX The amount to tweak the X position during close lineup
     * @param tweakY The amount to tweak the Y position during close lineup
     * @param lineupFeedback A function to call during close lineup with whether the controller is at its target. Can be
     *            null.
     * @return
     */
    public static Command autoScoreCommand(Drive drive, Vision vision, Arm arm, LEDs leds, ReefTarget target,
        Optional<BooleanSupplier> finishSequence, BooleanSupplier finishSequenceSlow, DoubleSupplier tweakX,
        DoubleSupplier tweakY, BooleanConsumer lineupFeedback, BooleanSupplier useArmLineup, boolean driveBackward) {
        double distanceAwayInches;
        if(target.level() == ReefLevel.L1) {
            distanceAwayInches = robotReefLineupL1Distance.get();
        } else if(target.level() == ReefLevel.L2) {
            distanceAwayInches = robotReefLineupL2Distance.get();
        } else if(target.level() == ReefLevel.L3) {
            distanceAwayInches = robotReefLineupL3Distance.get();
        } else {
            distanceAwayInches = robotReefLineupL4Distance.get();
        }

        Optional<BooleanSupplier> finish = finishSequence
            .map(f -> (() -> f.getAsBoolean() || finishSequenceSlow.getAsBoolean()));

        return Commands.sequence(//
            Commands.runOnce(() -> ScoringSequenceCommands.wristOverride = null), Commands.defer(() -> {
                return autoAlignSequence(drive, vision, leds, target, distanceAwayInches, false,
                    // During coarse lineup
                    Commands.none(),
                    // During close lineup
                    ScoringSequenceCommands.middleArmMovement(target.level(), arm), tweakX, tweakY, finish,
                    lineupFeedback).onlyIf(useArmLineup).withTimeout(DriverStation.isAutonomous() ? 4. : 10000);
            }, Set.of(drive)), //
            Commands.select(Map.of(//
                false,
                ScoringSequenceCommands
                    .scoreAtLevel(target.level(), arm, drive, target.branch().face.getFieldAngle(), !driveBackward)
                    .raceWith(leds.runStateCommand(LEDState.AutoScoring)).onlyIf(useArmLineup),
                true, ScoringSequenceCommands.scoreAtLevelSlowly(target.level(), arm)
                    .raceWith(leds.runStateCommand(LEDState.AutoScoring)).onlyIf(useArmLineup) //
            ), finishSequenceSlow::getAsBoolean)).withName("AutoScore" + target.toString());
    }

    /**
     * Gets a command that automatically scores at the selected level on the driver station interface. This is the
     * version of our automatic scoring mechanism intended for use in teleop.
     * @param drive The drive subsystem
     * @param vision The vision subsystem
     * @param arm The arm subsystem
     * @param leds The LEDs subsystem
     * @param finishSequence The close lineup waits for this to be true before finishing.
     * @param tweakX The amount to tweak the X position during close lineup
     * @param tweakY The amount to tweak the Y position during close lineup
     * @param lineupFeedback A function to call during close lineup with whether the controller is at its target. Can be
     *            null.
     * @return
     */
    public static Command autoScoreTeleopCommand(Drive drive, Vision vision, Arm arm, LEDs leds,
        BooleanSupplier finishSequence, BooleanSupplier finishSequenceSlow, DoubleSupplier tweakX,
        DoubleSupplier tweakY, BooleanConsumer lineupFeedback, BooleanSupplier useArmLineup) {
        AutoScoreState state = new AutoScoreState(DriverStationInterface.getInstance().getReefTarget());

        return new RestartWhenCommand(
            () -> autoScoreCommand(drive, vision, arm, leds, state.target, Optional.of(finishSequence),
                finishSequenceSlow, tweakX, tweakY, lineupFeedback, useArmLineup, true),

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
     * @param vision The vision subsystem
     * @param arm The arm subsystem
     * @param leds The LEDs subsystem
     * @param target The target to score at
     * @param finishSequence If present, the close lineup waits for this to be true before finishing. If not present,
     *            the close lineup finishes when the target is fully aligned.
     * @param tweakX The amount to tweak the X position during close lineup
     * @param tweakY The amount to tweak the Y position during close lineup
     * @return
     */
    public static Command removeAlgaeCommand(Drive drive, Vision vision, Arm arm, LEDs leds, ReefTarget target,
        Optional<BooleanSupplier> finishSequence, DoubleSupplier tweakX, DoubleSupplier tweakY) {
        Command autoAlign = autoAlignSequence(drive, vision, leds, target, 20, true,
            // During coarse lineup
            ScoringSequenceCommands.prepForAlgaeRemoval(target.level(), arm),
            // During close lineup
            Commands.none(), tweakX, tweakY, finishSequence, null);

        return Commands.sequence(//
            Commands.runOnce(() -> ScoringSequenceCommands.wristOverride = null), //
            autoAlign, //
            ScoringSequenceCommands.removeAlgae(target.level(), arm, drive, target.branch().face.getFieldAngle())
                .raceWith(leds.runStateCommand(LEDState.AutoScoring))//
        ).withName("RemoveAlgae" + target.toString());
    }

    /**
     * Gets a command that automatically removes algae at the selected level on the driver station interface.
     * @param drive The drive subsystem
     * @param vision The vision subsystem
     * @param arm The arm subsystem
     * @param leds The LEDs subsystem
     * @param finishSequence The close lineup waits for this to be true before finishing.
     * @param tweakX The amount to tweak the X position during close lineup
     * @param tweakY The amount to tweak the Y position during close lineup
     * @return
     */
    public static Command removeAlgaeTeleopCommand(Drive drive, Vision vision, Arm arm, LEDs leds,
        BooleanSupplier finishSequence, DoubleSupplier tweakX, DoubleSupplier tweakY) {
        AutoScoreState state = new AutoScoreState(DriverStationInterface.getInstance().getReefTarget());

        return new RestartWhenCommand(
            () -> removeAlgaeCommand(drive, vision, arm, leds, state.target, Optional.of(finishSequence), tweakX,
                tweakY),

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
