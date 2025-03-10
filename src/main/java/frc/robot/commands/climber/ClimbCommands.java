package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.util.Container;

public class ClimbCommands {
    private static final double CLIMB_SPEED_DEGREES = 20.;
    private static Container<Rotation2d> climberRotation = new Container<Rotation2d>(new Rotation2d());

    private ClimbCommands() {
    }

    public static Command resetClimbPosition() {
        return Commands.runOnce(() -> climberRotation.value = new Rotation2d());
    }

    public static Command climbCommand(Climber climber, DoubleSupplier climbSupplier) {
        return climber.run(() -> {
            climberRotation.value = climberRotation.value
                .plus(Rotation2d.fromDegrees(climbSupplier.getAsDouble() * CLIMB_SPEED_DEGREES * 0.02));

            climber.runClimber(climberRotation.value);
        });
    }
}
