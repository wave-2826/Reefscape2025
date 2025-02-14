package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.CloseLineupCommand;
import frc.robot.commands.util.RestartWhenCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.DriverStationInterface;
import frc.robot.util.ReefTarget;

public class AutoScoreCommands {
    /**
     * Gets a command that pathfinds to the target pose and precisely aligns to it. Because PathPlanner's default
     * pathfinding command is intended for long distances and doesn't move the robot if it's currently in the target
     * grid cell, we implement a second phase to align using a PID controller.
     */
    public static Command autoAlignCommand(Drive drive, Pose2d targetPose) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(DriveConstants.maxSpeedMetersPerSec, 5.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, constraints, MetersPerSecond.of(0.0));

        return Commands.sequence(pathfindingCommand, new CloseLineupCommand(drive, targetPose));
    }

    private static class AutoScoreState {
        public ReefTarget target;

        public AutoScoreState(ReefTarget target) {
            this.target = target;
        }
    }

    public static Command autoScoreCommand(Drive drive, Arm arm) {
        AutoScoreState state = new AutoScoreState(DriverStationInterface.getInstance().getReefTarget());

        var command = new RestartWhenCommand(
            Commands.parallel(Commands.defer(() -> autoAlignCommand(drive, state.target.branch().pose), Set.of(drive)), // Auto-align to the target
                Commands.defer(() -> arm.goToStateCommand(state.target.getArmState()), Set.of(arm)) // Move the arm to the target position
            ), () -> {
                var newTarget = DriverStationInterface.getInstance().getReefTarget();
                if(newTarget != state.target) {
                    // If the target has changed, restart the command
                    state.target = newTarget;
                    return true;
                }
                return false;
            });

        return command;
    }
}
