package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The intake subsystem. Manages all intake motors.
 */
public class Intake extends SubsystemBase {
    private static final double INTAKE_PITCH_TOLERANCE_DEGREES = 3.0;

    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private IntakeVisualizer visualizer = new IntakeVisualizer("intake");
    private double intakeSpeed = 0;
    private double transportSpeed = 0;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public Command setIntakePitch(Rotation2d pitch) {
        return Commands.run(() -> {
            Logger.recordOutput("Intake/TargetPitch", pitch.getRadians());
            io.setIntakePitch(pitch);
        }, this).until(this::atPitchSetpoint);
    }

    private boolean atPitchSetpoint() {
        return Math
            .abs(inputs.intakePitch.getDegrees() - inputs.intakePitch.getDegrees()) < INTAKE_PITCH_TOLERANCE_DEGREES;
    }

    /** Sets the intake pitch motor to coast. */
    public Command setIntakeCoast() {
        return Commands.runOnce(() -> {
            io.setIntakeCoast();
        }, this);
    }

    public Command runIntakeOpenLoop(double power) {
        return Commands.runOnce(() -> {
            intakeSpeed = power;
        }, this);
    }

    public Command setTransportOverrideSpeed(double power) {
        return Commands.runOnce(() -> {
            transportSpeed = power;
        }, this);
    }

    public void overridePitchPower(double power) {
        io.overridePitchPower(power);
    }

    public boolean intakeSensorTriggered() {
        return inputs.intakeSensorTriggered;
    }

    public boolean transportSensorTriggered() {
        return inputs.transortSensorTriggered;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        if(!DriverStation.isEnabled()) {
            transportSpeed = 0;
        }

        io.runVelocity(intakeSpeed, Math.max(intakeSpeed, transportSpeed));

        visualizer.update(inputs.intakePitch);
    }
}
