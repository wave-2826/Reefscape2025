package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

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

        RobotModeTriggers.disabled().onTrue(Commands.runOnce(io::resetToAbsolute, this));
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
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        visualizer.update(inputs.climberAbsolutePosition);

        climberDisconnectedAlert.set(!inputs.climberMotorConnected);
    }
}
