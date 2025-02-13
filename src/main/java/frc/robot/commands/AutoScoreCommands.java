package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.DriverStationInterface;
import frc.robot.util.ReefTarget;

public class AutoScoreCommands {
    public static Command autoAlignCommand(Drive drive, Pose2d target) {
        return Commands.sequence(new PathfindingCommand(target,
            new PathConstraints(MetersPerSecond.of(DriveConstants.maxSpeedMetersPerSec / 1.2),
                MetersPerSecondPerSecond.of(10.), RadiansPerSecond.of(10.), RadiansPerSecondPerSecond.of(10.)),
            null, null, null, null, null, null));
    }

    public static Command autoScoreCommand(Drive drive, Arm arm) {
        ReefTarget target = DriverStationInterface.getInstance().getReefTarget();
        Pose2d targetPose = target.branch().pose;
        var command = Commands.parallel(
            // Auto-align to the target
            autoAlignCommand(drive, targetPose));
        return command;
    }
}
