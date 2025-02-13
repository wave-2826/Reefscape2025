package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The arm subsystem. Manages the elevator, arm pitch, arm wrist rotation, and end effector. We combine all these into
 * one subsystem because they need to closely interact; most of the time, the elevator, arm pitch, and arm wrist move
 * simultaneously, and the end effector must work with the arm wrist while rotating because we use a coaxial drive
 * system for it.
 */
public class Arm extends SubsystemBase {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs;

    private final ArmVisualizer visualizer = new ArmVisualizer("arm");

    private ArmState targetState;

    private final Alert elevatorMotorDisconnectedAlert = new Alert("Elevator motor disconnected!", AlertType.kError);
    private final Alert armPitchMotorDisconnectedAlert = new Alert("Arm pitch motor disconnected!", AlertType.kError);
    private final Alert armWristMotorDisconnectedAlert = new Alert("Arm wrist motor disconnected!", AlertType.kError);
    private final Alert endEffectorMotorDisconnectedAlert = new Alert("End effector motor disconnected!",
        AlertType.kError);

    public Arm(ArmIO io) {
        this.io = io;
        this.inputs = new ArmIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        io.setElevatorHeight(targetState.height().in(Meters));
        io.setArmPitchPosition(targetState.pitch());
        io.setWristRotation(targetState.wristRotation());
        io.setEndEffectorState(targetState.endEffectorState());

        visualizer.update(inputs.elevatorHeightMeters, inputs.armPitchPosition, inputs.armWristPosition,
            inputs.gamePiecePresent);

        // Update alerts
        elevatorMotorDisconnectedAlert.set(!inputs.elevatorMotorsConnected);
        armPitchMotorDisconnectedAlert.set(!inputs.armPitchMotorConnected);
        armWristMotorDisconnectedAlert.set(!inputs.armWristMotorConnected);
        endEffectorMotorDisconnectedAlert.set(!inputs.endEffectorMotorConnected);
    }
}
