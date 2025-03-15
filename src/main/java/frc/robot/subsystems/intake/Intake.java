package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The intake subsystem. Manages all intake motors.
 */
public class Intake extends SubsystemBase {
    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public void setIntakePitch(Rotation2d pitch) {
        Logger.recordOutput("Intake/TargetPitch", pitch.getRadians());
        io.setIntakePitch(pitch);
    }

    public void runIntakeOpenLoop(double power) {
        io.runIntakeOpenLoop(power);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }
}
