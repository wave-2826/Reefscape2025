package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmState.WristRotation;
import frc.robot.util.LoggedTunableNumber;

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

    private ArmState targetState = new ArmState(Rotation2d.fromDegrees(0.0), Meters.of(0.0), WristRotation.Vertical,
        new EndEffectorState(EndEffectorState.Mode.Hold));

    private final Alert elevatorMotorDisconnectedAlert = new Alert("Elevator motor disconnected!", AlertType.kError);
    private final Alert elevatorHeightSensorDisconnectedAlert = new Alert("Elevator height sensor disconnected!",
        AlertType.kError);
    private final Alert armPitchMotorDisconnectedAlert = new Alert("Arm pitch motor disconnected!", AlertType.kError);
    private final Alert armWristMotorDisconnectedAlert = new Alert("Arm wrist motor disconnected!", AlertType.kError);
    private final Alert endEffectorMotorDisconnectedAlert = new Alert("End effector motor disconnected!",
        AlertType.kError);

    private static final LoggedTunableNumber armPitchKg = new LoggedTunableNumber("Arm/pitchKg");

    static {
        armPitchKg.initDefault(ArmConstants.ShoulderConstants.armPitchKg);
    }

    public Arm(ArmIO io) {
        this.io = io;
        this.inputs = new ArmIOInputsAutoLogged();
    }

    public Command goToStateCommand(ArmState state) {
        return Commands.runOnce(() -> {
            targetState = state;
        }, this).until(this::isAtTarget).handleInterrupt(() -> {
            // If we are interrupted, set the target state to the current state
            targetState = new ArmState(inputs.armPitchPosition, Meters.of(inputs.elevatorHeightMeters),
                WristRotation.Vertical, new EndEffectorState(EndEffectorState.Mode.Hold));
        });
    }

    public Command setTargetStateCommand(Supplier<ArmState> state) {
        return Commands.run(() -> {
            targetState = state.get();
        });
    }

    private static final double TARGET_HEIGHT_TOLERANCE_METERS = 0.05;
    private static final double TARGET_PITCH_TOLERANCE_DEGREES = 2.0;
    private static final double TARGET_WRIST_TOLERANCE_DEGREES = 2.0;

    private boolean isAtTarget() {
        return Math.abs(inputs.elevatorHeightMeters - targetState.height().in(Meters)) < TARGET_HEIGHT_TOLERANCE_METERS
            && Math.abs(inputs.armPitchPosition.getDegrees()
                - targetState.pitch().getDegrees()) < TARGET_PITCH_TOLERANCE_DEGREES
            && Math.abs(inputs.armWristPosition.getDegrees()
                - targetState.wristRotation().rotation.getDegrees()) < TARGET_WRIST_TOLERANCE_DEGREES;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        io.setElevatorHeight(targetState.height().in(Meters));
        io.setArmPitchPosition(targetState.pitch(),
            Math.abs(Math.cos(inputs.armPitchPosition.getRadians())) * armPitchKg.get());
        io.setWristRotation(targetState.wristRotation());
        io.setEndEffectorState(targetState.endEffectorState());

        Logger.recordOutput("Arm/TargetHeight", targetState.height().in(Meters));
        Logger.recordOutput("Arm/TargetPitch", targetState.pitch().getDegrees());
        Logger.recordOutput("Arm/TargetWrist", targetState.wristRotation().rotation.getDegrees());
        Logger.recordOutput("Arm/TargetEndEffector", targetState.endEffectorState().getVelocityControl().orElse(0.0));
        Logger.recordOutput("Arm/AtTarget", isAtTarget());

        visualizer.update(inputs.elevatorHeightMeters, inputs.armPitchPosition, inputs.armWristPosition,
            inputs.gamePiecePresent);

        // Update alerts
        elevatorMotorDisconnectedAlert.set(!inputs.elevatorMotorsConnected);
        elevatorHeightSensorDisconnectedAlert.set(!inputs.elevatorHeightSensorConnected);
        armPitchMotorDisconnectedAlert.set(!inputs.armPitchMotorConnected);
        armWristMotorDisconnectedAlert.set(!inputs.armWristMotorConnected);
        endEffectorMotorDisconnectedAlert.set(!inputs.endEffectorMotorConnected);
    }
}
