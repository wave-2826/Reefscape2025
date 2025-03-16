package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.Container;

public class IntakeCommands {
    public static Command intakeCommand(Intake intake, DoubleSupplier pitchStick, DoubleSupplier powerStick) {
        var pitch = new Container<Rotation2d>(Rotation2d.kZero);
        return Commands.run(() -> {
            pitch.value = Rotation2d.fromDegrees(
                pitch.value.getDegrees() - MathUtil.applyDeadband(pitchStick.getAsDouble(), 0.2) * 180 * 0.02);
            intake.setIntakePitch(pitch.value);
            intake.runIntakeOpenLoop(MathUtil.applyDeadband(powerStick.getAsDouble(), 0.2));
        }, intake);
    }
}
