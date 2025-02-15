package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Set;

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
     * The distance from the reef branch to the center of the robot when lining up to score, in meters.
     */
    private static final LoggedTunableNumber robotReefLineupDistance = new LoggedTunableNumber(
        "AutoScore/ReefLineupDistance", 0.5);

    /**
     * Gets a command that pathfinds to the target pose and precisely aligns to it. Because PathPlanner's default
     * pathfinding command is intended for long distances and doesn't move the robot if it's currently in the target
     * grid cell, we implement a second phase to align using a PID controller.
     */
    public static Command autoAlignCommand(Drive drive, Pose2d targetPose) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(DriveConstants.maxSpeedMetersPerSec, 5.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

        Pose2d currentPose = drive.getPose();

        // The pose to pathfind to is slightly further outward than the target pose to ensure we don't get stuck
        Pose2d pathfindPose = targetPose.transformBy(new Transform2d(Units.inchesToMeters(5), 0.0, new Rotation2d()));
        // A pose to initially pathfind to if we're near the reef
        Pose2d safeReefPose = new Pose2d(
            currentPose.getTranslation()
                .plus(new Translation2d(Units.inchesToMeters(15), 0.0)
                    .rotateBy(currentPose.getTranslation().minus(FieldConstants.reefCenter).getAngle())),
            currentPose.getRotation());

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return Commands.sequence(
            // Move slightly outward first if we're near the reef
            AutoBuilder.pathfindToPoseFlipped(safeReefPose, constraints, MetersPerSecond.of(0.0)).onlyIf(() -> {
                return currentPose.getTranslation().getDistance(FieldConstants.reefCenter) < Units.inchesToMeters(40);
            }), AutoBuilder.pathfindToPoseFlipped(pathfindPose, constraints, MetersPerSecond.of(0.0)), // Move to the target pose
            Commands.runOnce(() -> Logger.recordOutput("AutoScore/RunningCloseLineup", true)),
            new CloseLineupCommand(drive, targetPose) // Final adjustment
        ).finallyDo(() -> {
            Logger.recordOutput("AutoScore/RunningCloseLineup", false);
        });
    }

    private static class AutoScoreState {
        public ReefTarget target;

        public AutoScoreState(ReefTarget target) {
            this.target = target;
        }

        public Pose2d getLineupPose() {
            return target.branch().pose
                .transformBy(new Transform2d(robotReefLineupDistance.get(), 0.0, Rotation2d.fromDegrees(180)));
        }
    }

    public static Command autoScoreCommand(Drive drive, Arm arm) {
        return Commands.defer(() -> {
            AutoScoreState state = new AutoScoreState(DriverStationInterface.getInstance().getReefTarget());
            int id = state.target.hashCode();

            return new RestartWhenCommand(Commands.parallel(Commands.runOnce(() -> {
                Logger.recordOutput("AutoScore/TargetPose", state.getLineupPose());
            }), Commands.defer(() -> autoAlignCommand(drive, state.getLineupPose()), Set.of(drive)), // Auto-align to the target
                Commands.defer(() -> arm.goToStateCommand(state.target.getArmState()), Set.of(arm)) // Move the arm to the target position
            ), () -> {
                var newTarget = DriverStationInterface.getInstance().getReefTarget();
                if(!newTarget.equals(state.target) || robotReefLineupDistance.hasChanged(id)) {
                    // If the target has changed, restart the command
                    state.target = newTarget;
                    System.out.println("Target changed to " + newTarget);
                    return true;
                }
                return false;
            });
        }, Set.of(drive, arm)).finallyDo(() -> {
            Logger.recordOutput("AutoScore/TargetPose", new Pose2d());
        });
    }
}
