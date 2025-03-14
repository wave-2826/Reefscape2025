package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.commands.arm.ScoringSequenceCommands;
import frc.robot.commands.drive.CloseLineupCommand;
import frc.robot.commands.util.RestartWhenCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.DriverStationInterface;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ReefTarget;

public class AutoScoreCommands {
    /**
     * The distance from the reef branch to the center of the robot when lining up to score L1 in meters.
     */
    private static final LoggedTunableNumber robotReefLineupL1Distance = new LoggedTunableNumber(
        "AutoScore/L1ReefLineupDistance", 0.75);

    /**
     * The distance from the reef branch to the center of the robot when lining up to score L2-L4 in meters.
     */
    private static final LoggedTunableNumber robotReefLineupBranchDistance = new LoggedTunableNumber(
        "AutoScore/BranchReefLineupDistance", 0.6);

    /**
     * The amount the driver can tweak the auto lineup position, in inches.
     */
    private static final LoggedTunableNumber autoAlignTweakAmount = new LoggedTunableNumber(
        "AutoScore/AutoAlignTweakInches", 3.0);

    /**
     * Gets a command that pathfinds to the target pose and precisely aligns to it. Because PathPlanner's default
     * pathfinding command is intended for long distances and doesn't move the robot if it's currently in the target
     * grid cell, we implement a second phase to align using a PID controller. Calls the passed commands during
     * different lineup stages.
     */
    public static Command autoAlignSequence(Drive drive, Pose2d targetPose, Command coarseLineupCommand,
        Command closeLineupCommand, DoubleSupplier tweakX, DoubleSupplier tweakY) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(DriveConstants.maxSpeedMetersPerSec, 5.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

        Pose2d currentPose = drive.getPose();

        // The pose to pathfind to is slightly further outward than the target pose to ensure we don't get stuck
        Pose2d pathfindPose = targetPose.transformBy(new Transform2d(Units.inchesToMeters(-3), 0.0, Rotation2d.kZero));
        // A pose to initially pathfind to if we're near the reef
        Pose2d safeReefPose = new Pose2d(
            currentPose.getTranslation()
                .plus(new Translation2d(Units.inchesToMeters(15), 0.0)
                    .rotateBy(currentPose.getTranslation().minus(FieldConstants.reefCenter).getAngle())),
            currentPose.getRotation());

        Supplier<Pose2d> getTarget = () -> targetPose.plus(
            new Transform2d(new Translation2d(-tweakX.getAsDouble() * Units.inchesToMeters(autoAlignTweakAmount.get()),
                tweakY.getAsDouble() * Units.inchesToMeters(autoAlignTweakAmount.get())), Rotation2d.kZero));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        // @formatter:off
        return Commands.sequence(
            // Move slightly outward first if we're near the reef
            AutoBuilder.pathfindToPoseFlipped(safeReefPose, constraints, MetersPerSecond.of(0.0)).onlyIf(() -> {
                return currentPose.getTranslation().getDistance(FieldConstants.reefCenter) < Units.inchesToMeters(40);
            }),
            Commands.parallel(
                AutoBuilder.pathfindToPoseFlipped(pathfindPose, constraints, MetersPerSecond.of(0.0)), // Move to the target pose
                coarseLineupCommand
            ),
            Commands.runOnce(() -> Logger.recordOutput("AutoScore/RunningCloseLineup", true)),
            Commands.parallel(
                closeLineupCommand, // Run during final adjustment
                new CloseLineupCommand(drive, getTarget) // Final adjustment
            )
        ).finallyDo(() -> {
            Logger.recordOutput("AutoScore/RunningCloseLineup", false);
        }); // @formatter:on
    }

    private static class AutoScoreState {
        public ReefTarget target;

        public AutoScoreState(ReefTarget target) {
            this.target = target;
        }

        public Pose2d getLineupPose(boolean isL1) {
            return target.branch().pose.transformBy(
                new Transform2d(isL1 ? robotReefLineupL1Distance.get() : robotReefLineupBranchDistance.get(), 0.0,
                    Rotation2d.fromDegrees(180)));
        }
    }

    public static Command autoScoreStartCommand(Drive drive, Arm arm, BooleanSupplier finishSequene,
        DoubleSupplier tweakX, DoubleSupplier tweakY) {
        AutoScoreState state = new AutoScoreState(DriverStationInterface.getInstance().getReefTarget());
        int id = state.hashCode();

        return new RestartWhenCommand( // @formatter:off Our formatter makes this far less readable
            () -> {
                Pose2d lineupPose = state.getLineupPose(state.target.level() == ReefLevel.L1);
                return Commands.sequence(
                    Commands.runOnce(() -> {
                        Logger.recordOutput("AutoScore/TargetPose", lineupPose);
                    }), 
                    autoAlignSequence(
                        drive,
                        lineupPose,
                        // During coarse lineup
                        ScoringSequenceCommands.prepForScoring(state.target.level(), arm),
                        // During close lineup
                        ScoringSequenceCommands.middleArmMovement(state.target.level(), arm),
                        tweakX, tweakY
                    ).until(finishSequene), // Auto-align to the target
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
                    System.out.println("Target changed to " + newTarget);
                    return true;
                }
                return false;
            }, Set.of(drive, arm)).finallyDo(() -> {
                Logger.recordOutput("AutoScore/TargetPose", Pose2d.kZero);
            });
    }
}
