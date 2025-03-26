package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

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
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.DriverStationInterface;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ReefTarget;

public class AutoScoreCommands {
    /**
     * The distance from the reef branch to the center of the robot when lining up to score L1 in meters.
     */
    private static final LoggedTunableNumber robotReefLineupL1Distance = new LoggedTunableNumber(//
        "AutoScore/L1ReefLineupDistance", 35.5);

    /**
     * The distance from the reef branch to the center of the robot when lining up to score L2-L4 in meters.
     */
    private static final LoggedTunableNumber robotReefLineupBranchDistance = new LoggedTunableNumber(//
        "AutoScore/BranchReefLineupDistance", 24.40);

    /**
     * The distance from the reef branch to the center of the robot when lining up to score L4 in meters.
     */
    private static final LoggedTunableNumber robotReefLineupL4Distance = new LoggedTunableNumber(//
        "AutoScore/L4ReefLineupDistance", 22.5);

    /**
     * The amount the driver can tweak the auto lineup position, in inches.
     */
    private static final LoggedTunableNumber autoAlignTweakAmount = new LoggedTunableNumber(//
        "AutoScore/AutoAlignTweakInches", 4.0);

    /**
     * A tweaking factor added to our horizontal lineup distance from the center between two branches.
     */
    private static final LoggedTunableNumber centerDistanceTweak = new LoggedTunableNumber(//
        "AutoScore/CenterDistanceTweak", -0.75);

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
    public static Command autoAlignSequence(Drive drive, Vision vision, ReefTarget target, Command coarseLineupCommand,
        Command closeLineupCommand, DoubleSupplier tweakX, DoubleSupplier tweakY,
        Optional<BooleanSupplier> finishSequence, BooleanConsumer lineupFeedback) {
        // Create the constraints to use while pathfinding
        // PathConstraints constraints = new PathConstraints(DriveConstants.maxSpeedMetersPerSec, 5.0,
        //     Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Pose2d currentPose = drive.getPose();

        boolean isLeft = target.branch().isLeft;

        double distanceAwayInches;
        if(target.level() == ReefLevel.L1) {
            distanceAwayInches = robotReefLineupL1Distance.get();
        } else if(target.level() == ReefLevel.L4) {
            distanceAwayInches = robotReefLineupL4Distance.get();
        } else {
            distanceAwayInches = robotReefLineupBranchDistance.get();
        }

        double centerDistance = FieldConstants.reefBranchSeparation.in(Meters) / 2.
            + Units.inchesToMeters(centerDistanceTweak.get());
        Transform2d tagRelativeOffset = new Transform2d(
            new Translation2d(Units.inchesToMeters(distanceAwayInches), isLeft ? -centerDistance : centerDistance),
            Rotation2d.k180deg);

        ReefFace reefFace = target.branch().face;

        Pose2d initialLineupPosition = reefFace.tagPose.transformBy(tagRelativeOffset)
            .plus(new Transform2d(new Translation2d(Units.inchesToMeters(-5.), 0.), Rotation2d.kZero));

        // A pose to initially pathfind to if we're near the reef
        // Pose2d safeReefPose = new Pose2d(
        //     currentPose.getTranslation()
        //         .plus(new Translation2d(Units.inchesToMeters(15), 0.0)
        //             .rotateBy(currentPose.getTranslation().minus(FieldConstants.reefCenter).getAngle())),
        //     currentPose.getRotation());

        Supplier<Transform2d> getFieldRelativeOffset = () -> new Transform2d(
            new Translation2d(-tweakY.getAsDouble() * Units.inchesToMeters(autoAlignTweakAmount.get()),
                -tweakX.getAsDouble() * Units.inchesToMeters(autoAlignTweakAmount.get())),
            Rotation2d.kZero);

        // HACK ..?
        // BooleanSupplier finishEarly = finishSequence.isEmpty() ? () -> false : finishSequence.get();

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        // @formatter:off
        return Commands.sequence(
            Commands.runOnce(() -> {
                Logger.recordOutput("AutoScore/InitialLineupPose", initialLineupPosition);
            }), 

            // Move slightly outward first if we're near the reef
            // AutoBuilder.pathfindToPoseFlipped(safeReefPose, constraints, MetersPerSecond.of(0.0)).until(finishEarly).onlyIf(() -> {
            //     return currentPose.getTranslation().getDistance(FieldConstants.reefCenter) < Units.inchesToMeters(40);
            // }),
            // Commands.parallel(
            //     AutoBuilder.pathfindToPoseFlipped(initialLineupPosition, constraints, MetersPerSecond.of(0.0)), // Move to the target pose
            //     coarseLineupCommand.withTimeout(3)
            // ).until(finishEarly).onlyIf(() -> {
            //     return currentPose.getTranslation().getDistance(initialLineupPosition.getTranslation()) > Units.inchesToMeters(15);
            // }),
            // Commands.waitUntil(() -> !finishEarly.getAsBoolean()),
            Commands.runOnce(() -> Logger.recordOutput("AutoScore/RunningCloseLineup", true)),
            Commands.parallel(
                closeLineupCommand.withTimeout(2.0), // Run during final adjustment
                // TODO: Fully line up before finishing if the finish sequence button is held
                new CloseLineupCommand(drive, vision, reefFace.getAprilTagID(), tagRelativeOffset, getFieldRelativeOffset, finishSequence, lineupFeedback) // Final adjustment
            )
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
     * @param drive The drive subsystem
     * @param vision The vision subsystem
     * @param arm The arm subsystem
     * @param target The target to score at
     * @return
     */
    public static Command autoScoreCommand(Drive drive, Vision vision, Arm arm, ReefTarget target) {
        return autoScoreCommand(drive, vision, arm, target, Optional.empty(), () -> 0, () -> 0, null, () -> true);
    }

    /**
     * Gets a command that automatically scores at the selected level on the driver station interface. This is the
     * version of our automatic scoring mechanism intended for use in autonomous.
     * @param drive The drive subsystem
     * @param vision The vision subsystem
     * @param arm The arm subsystem
     * @param target The target to score at
     * @param finishSequence If present, the close lineup waits for this to be true before finishing. If not present,
     *            the close lineup finishes when the target is fully aligned.
     * @param tweakX The amount to tweak the X position during close lineup
     * @param tweakY The amount to tweak the Y position during close lineup
     * @param lineupFeedback A function to call during close lineup with whether the controller is at its target. Can be
     *            null.
     * @return
     */
    public static Command autoScoreCommand(Drive drive, Vision vision, Arm arm, ReefTarget target,
        Optional<BooleanSupplier> finishSequence, DoubleSupplier tweakX, DoubleSupplier tweakY,
        BooleanConsumer lineupFeedback, BooleanSupplier useArmLineup) {
        Command autoAlign = autoAlignSequence(drive, vision, target,
            // During coarse lineup
            ScoringSequenceCommands.prepForScoring(target.level(), arm).onlyIf(useArmLineup),
            // During close lineup
            ScoringSequenceCommands.middleArmMovement(target.level(), arm), tweakX, tweakY, finishSequence,
            lineupFeedback).onlyIf(useArmLineup);

        return Commands
            .sequence(autoAlign, ScoringSequenceCommands.scoreAtLevel(target.level(), arm, drive).onlyIf(useArmLineup))
            .withName("AutoScore" + target.toString());
    }

    /**
     * Gets a command that automatically scores at the selected level on the driver station interface. This is the
     * version of our automatic scoring mechanism intended for use in teleop.
     * @param drive The drive subsystem
     * @param vision The vision subsystem
     * @param arm The arm subsystem
     * @param finishSequence The close lineup waits for this to be true before finishing.
     * @param tweakX The amount to tweak the X position during close lineup
     * @param tweakY The amount to tweak the Y position during close lineup
     * @param lineupFeedback A function to call during close lineup with whether the controller is at its target. Can be
     *            null.
     * @return
     */
    public static Command autoScoreTeleopCommand(Drive drive, Vision vision, Arm arm, BooleanSupplier finishSequence,
        DoubleSupplier tweakX, DoubleSupplier tweakY, BooleanConsumer lineupFeedback, BooleanSupplier useArmLineup) {
        AutoScoreState state = new AutoScoreState(DriverStationInterface.getInstance().getReefTarget());
        int id = state.hashCode();

        return new RestartWhenCommand(
            () -> autoScoreCommand(drive, vision, arm, state.target, Optional.of(finishSequence), tweakX, tweakY,
                lineupFeedback, useArmLineup),

            // Restart when our target changes
            () -> {
                var newTarget = DriverStationInterface.getInstance().getReefTarget();
                if(!newTarget.equals(state.target) || robotReefLineupL1Distance.hasChanged(id)
                    || robotReefLineupBranchDistance.hasChanged(id)) {
                    state.target = newTarget;
                    return true;
                }
                return false;
            }, Set.of(drive, arm));
    }
}
