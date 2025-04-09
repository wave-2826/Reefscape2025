package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.util.Container;

public class ClimbCommands {
    private static Container<Rotation2d> climberRotation = new Container<Rotation2d>(Rotation2d.kZero);

    private ClimbCommands() {
    }

    public static Command resetClimbPosition() {
        return Commands.runOnce(() -> climberRotation.value = Rotation2d.kZero);
    }

    public static Command climbCommand(Climber climber, DoubleSupplier climbSupplier) {
        return Commands.run(() -> {
            double climbSpeed = MathUtil.applyDeadband(climbSupplier.getAsDouble(), 0.2);
            if(climber.getPitch().getDegrees() < 8) {
                climbSpeed = Math.min(0, climbSpeed);
            }
            if(climber.getPitch().getDegrees() > 90) {
                climbSpeed = Math.max(0, climbSpeed);
            }

            climber.runClimberOpenLoop(climbSpeed);
        }, climber).finallyDo(() -> climber.runClimberOpenLoop(0));
    }
}
