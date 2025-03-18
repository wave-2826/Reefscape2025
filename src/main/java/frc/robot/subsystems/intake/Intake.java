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

    public Command setIntakePitchCommand(Rotation2d pitch) {
        return Commands.run(() -> {
            setIntakePitchCommand(pitch);
        }, this).until(this::atPitchSetpoint);
    }

    public void setIntakePitch(Rotation2d pitch) {
        Logger.recordOutput("Intake/TargetPitch", pitch.getRadians());
        io.setIntakePitch(pitch);
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

    public Command runIntakeOpenLoopCommand(double power) {
        return Commands.runOnce(() -> {
            runIntakeOpenLoop(power);
        }, this);
    }

    public void runIntakeOpenLoop(double power) {
        intakeSpeed = power;
    }

    public Command setTransportOverrideSpeedCommand(double power) {
        return Commands.runOnce(() -> {
            setTransportOverrideSpeed(power);
        }, this);
    }

    public void setTransportOverrideSpeed(double power) {
        transportSpeed = power;
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

        io.runVelocity(intakeSpeed, transportSpeed == 0 ? intakeSpeed : transportSpeed);

        visualizer.update(inputs.intakePitch);
    }
}
