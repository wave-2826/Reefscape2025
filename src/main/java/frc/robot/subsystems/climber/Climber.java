package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The climber subsystem. Manages the climber motor and resetting.
 */
public class Climber extends SubsystemBase {
    ClimberIO io;
    ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    ClimberVisualizer visualizer = new ClimberVisualizer("climber");

    private final Alert climberDisconnectedAlert = new Alert("Climber motor disconnected!", AlertType.kError);

    public Climber(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        visualizer.update(inputs.climberPosition);

        climberDisconnectedAlert.set(!inputs.climberMotorConnected);
    }
}
