package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

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
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.DriverStationInterface;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ReefTarget;

public class AutoScoreCommands {
    /**
     * The distance from the reef branch to the center of the robot when lining up to score L1 in meters.
     */
    private static final LoggedTunableNumber robotReefLineupL1Distance = new LoggedTunableNumber(
        "AutoScore/L1ReefLineupDistance", 0.8);

    /**
     * The distance from the reef branch to the center of the robot when lining up to score L2-L4 in meters.
     */
    private static final LoggedTunableNumber robotReefLineupBranchDistance = new LoggedTunableNumber(
        "AutoScore/BranchReefLineupDistance", 0.65);

    /**
     * The amount the driver can tweak the auto lineup position, in inches.
     */
    private static final LoggedTunableNumber autoAlignTweakAmount = new LoggedTunableNumber(
        "AutoScore/AutoAlignTweakInches", 5.0);

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
        PathConstraints constraints = new PathConstraints(DriveConstants.maxSpeedMetersPerSec, 5.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

        Pose2d currentPose = drive.getPose();

        boolean isLeft = target.branch().isLeft;
        double distanceAway = target.level() == ReefLevel.L1 ? robotReefLineupL1Distance.get()
            : robotReefLineupBranchDistance.get();

        Transform2d tagRelativeOffset = new Transform2d(
            new Translation2d(distanceAway, isLeft ? -FieldConstants.reefBranchSeparation.in(Meters) / 2.
                : FieldConstants.reefBranchSeparation.in(Meters) / 2.),
            Rotation2d.k180deg);

        ReefFace reefFace = target.branch().face;

        Pose2d initialLineupPosition = reefFace.tagPose.transformBy(tagRelativeOffset)
            .plus(new Transform2d(new Translation2d(Units.inchesToMeters(-5.), 0.), Rotation2d.kZero));

        // A pose to initially pathfind to if we're near the reef
        Pose2d safeReefPose = new Pose2d(
            currentPose.getTranslation()
                .plus(new Translation2d(Units.inchesToMeters(15), 0.0)
                    .rotateBy(currentPose.getTranslation().minus(FieldConstants.reefCenter).getAngle())),
            currentPose.getRotation());

        Supplier<Transform2d> getFieldRelativeOffset = () -> new Transform2d(
            new Translation2d(-tweakY.getAsDouble() * Units.inchesToMeters(autoAlignTweakAmount.get()),
                -tweakX.getAsDouble() * Units.inchesToMeters(autoAlignTweakAmount.get())),
            Rotation2d.kZero);

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        // @formatter:off
        return Commands.sequence(
            Commands.runOnce(() -> {
                Logger.recordOutput("AutoScore/InitialLineupPose", initialLineupPosition);
            }), 

            // Move slightly outward first if we're near the reef
            AutoBuilder.pathfindToPoseFlipped(safeReefPose, constraints, MetersPerSecond.of(0.0)).onlyIf(() -> {
                return currentPose.getTranslation().getDistance(FieldConstants.reefCenter) < Units.inchesToMeters(40);
            }),
            Commands.parallel(
                AutoBuilder.pathfindToPoseFlipped(initialLineupPosition, constraints, MetersPerSecond.of(0.0)), // Move to the target pose
                coarseLineupCommand.withTimeout(3)
            ),
            Commands.runOnce(() -> Logger.recordOutput("AutoScore/RunningCloseLineup", true)),
            Commands.parallel(
                closeLineupCommand, // Run during final adjustment
                // TODO: Fully line up before finishing if the finish sequence button is held
                new CloseLineupCommand(drive, vision, reefFace.getAprilTagID(), tagRelativeOffset, getFieldRelativeOffset, finishSequence, lineupFeedback) // Final adjustment
            )
        ).finallyDo(() -> {
            Logger.recordOutput("AutoScore/RunningCloseLineup", false);
            Logger.recordOutput("AutoScore/InitialLineupPose", Pose2d.kZero);
        }); // @formatter:on
    }

    private static class AutoScoreState {
        public ReefTarget target;

        public AutoScoreState(ReefTarget target) {
            this.target = target;
        }
    }

    /**
     * Gets a command that scores at the given level. This command will automatically align to the target and score.
     * @param drive The drive subsystem
     * @param vision The vision subsystem
     * @param arm The arm subsystem
     * @param finishSequence If present, the close lineup waits for this to be true before finishing. If not present,
     *            the close lineup finishes when the target is fully aligned.
     * @param tweakX The amount to tweak the X position during close lineup
     * @param tweakY The amount to tweak the Y position during close lineup
     * @param lineupFeedback A function to call during close lineup with whether the controller is at its target. Can be
     *            null.
     * @return
     */
    public static Command autoScoreCommand(Drive drive, Vision vision, Arm arm,
        Optional<BooleanSupplier> finishSequence, DoubleSupplier tweakX, DoubleSupplier tweakY,
        BooleanConsumer lineupFeedback) {
        AutoScoreState state = new AutoScoreState(DriverStationInterface.getInstance().getReefTarget());
        int id = state.hashCode();

        return new RestartWhenCommand( // @formatter:off Our formatter makes this far less readable
            () -> {
                Command autoAlign = autoAlignSequence(
                    drive, vision,
                    state.target,
                    // During coarse lineup
                    ScoringSequenceCommands.prepForScoring(state.target.level(), arm),
                    // During close lineup
                    ScoringSequenceCommands.middleArmMovement(state.target.level(), arm),
                    tweakX, tweakY,
                    finishSequence, lineupFeedback
                );
                
                return Commands.sequence(
                    autoAlign,
                    // Score
                    ScoringSequenceCommands.scoreAtLevel(state.target.level(), arm, drive)
                );
            }, // @formatter:on

            // Restart when our target changes
            () -> {
                var newTarget = DriverStationInterface.getInstance().getReefTarget();
                if(!newTarget.equals(state.target) || robotReefLineupL1Distance.hasChanged(id)
                    || robotReefLineupBranchDistance.hasChanged(id)) {
                    state.target = newTarget;
                    // TODO: Test this functionality more extensively
                    System.out.println("Target changed to " + newTarget);
                    return true;
                }
                return false;
            }, Set.of(drive, arm));
    }
}
