package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.util.LoggedTracer;

/**
 * The climber subsystem. Manages the climber motor and resetting.
 */
public class Climber extends SubsystemBase {
    ClimberIO io;
    ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    ClimberVisualizer visualizer = new ClimberVisualizer("climber");

    private final Alert climberDisconnectedAlert = new Alert("Climber motor disconnected!", AlertType.kError);
    private final Alert climberEncoderDisconnectedAlert = new Alert("Climber encoder reading 0!", AlertType.kWarning);

    /**
     * The pitch of the climber. 0 is fully back and positive numbers move outward.
     * @return
     */
    public Rotation2d getPitch() {
        return inputs.climberAbsolutePosition;
    }

    public Climber(ClimberIO io) {
        this.io = io;

        RobotModeTriggers.disabled().onFalse(Commands.runOnce(io::resetToAbsolute, this))
            .onTrue(Commands.runOnce(() -> runClimberOpenLoop(0.0)));
    }

    public void runClimber(Rotation2d position) {
        io.setClimberBrakeMode(true);
        io.setClimberTargetAngle(position);
    }

    public void runClimberOpenLoop(double power) {
        io.runClimberOpenLoop(power);
    }

    public void disableClimber() {
        // TODO: Call this??
        io.setClimberBrakeMode(false);
        io.runClimberOpenLoop(0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        visualizer.update(inputs.climberAbsolutePosition);

        climberDisconnectedAlert.set(!inputs.climberMotorConnected);
        climberEncoderDisconnectedAlert.set(inputs.climberAbsolutePosition.getRadians() == 0);

        LoggedTracer.record("Climber");
    }
}
