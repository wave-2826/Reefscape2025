package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.util.Container;

public class ClimbCommands {
    // private static final double CLIMB_SPEED_DEGREES = 20.;
    private static Container<Rotation2d> climberRotation = new Container<Rotation2d>(Rotation2d.kZero);

    private ClimbCommands() {
    }

    public static Command resetClimbPosition() {
        return Commands.runOnce(() -> climberRotation.value = Rotation2d.kZero);
    }

    public static Command climbCommand(Climber climber, DoubleSupplier climbSupplier, Intake intake) {
        return Commands.run(() -> {
            double climbSpeed = MathUtil.applyDeadband(climbSupplier.getAsDouble(), 0.2);
            // climberRotation.value = climberRotation.value
            //     .plus(Rotation2d.fromDegrees(climbSpeed * CLIMB_SPEED_DEGREES * 0.02));

            // climber.runClimber(climberRotation.value);
            if(climber.getPitch().getDegrees() < 5) {
                climbSpeed = Math.min(0, climbSpeed);
            }
            if(climber.getPitch().getDegrees() > 93) {
                climbSpeed = Math.max(0, climbSpeed);
            }

            climber.runClimberOpenLoop(climbSpeed);

            intake.setIntakeState(IntakeState.Climb);
        }, climber, intake).finallyDo(() -> climber.runClimberOpenLoop(0));
    }
}
