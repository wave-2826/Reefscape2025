package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Meters;

import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmState.WristRotation;
import frc.robot.util.LoggedTracer;
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

    private ArmState targetState = null;
    private ArmState adjustedTarget = null;

    private final Alert elevatorMotorDisconnectedAlert = new Alert("Elevator motor disconnected!", AlertType.kError);
    private final Alert elevatorHeightSensorDisconnectedAlert = new Alert("Elevator height sensor disconnected!",
        AlertType.kError);
    private final Alert armPitchMotorDisconnectedAlert = new Alert("Arm pitch motor disconnected!", AlertType.kError);
    private final Alert armWristMotorDisconnectedAlert = new Alert("Arm wrist motor disconnected!", AlertType.kError);
    private final Alert endEffectorMotorDisconnectedAlert = new Alert("End effector motor disconnected!",
        AlertType.kError);

    private static final LoggedTunableNumber elevatorKg = new LoggedTunableNumber("Arm/elevatorKg");
    private static final LoggedTunableNumber armPitchKg = new LoggedTunableNumber("Arm/pitchKg");

    static {
        elevatorKg.initDefault(Constants.isSim ? ArmConstants.ElevatorConstants.elevatorKgSim
            : ArmConstants.ElevatorConstants.elevatorKgReal);
        armPitchKg.initDefault(Constants.isSim ? ArmConstants.ShoulderConstants.armPitchKgSim
            : ArmConstants.ShoulderConstants.armPitchKgReal);
    }

    public Arm(ArmIO io) {
        this.io = io;
        this.inputs = new ArmIOInputsAutoLogged();
    }

    public Command goToStateCommand(ArmState state) {
        return Commands.run(() -> {
            targetState = state;
            adjustedTarget = getAdjustedTarget();
        }, this).until(this::isAtTarget).withName("ArmGoToState");
    }

    public Command goToStateCommand(Supplier<ArmState> state) {
        return Commands.defer(() -> goToStateCommand(state.get()), Set.of(this));
    }

    public Command setTargetStateCommand(Supplier<ArmState> state) {
        return Commands.run(() -> {
            targetState = state.get();
        });
    }

    private static final double TARGET_HEIGHT_TOLERANCE_METERS = Units.inchesToMeters(0.25);
    private static final double TARGET_PITCH_TOLERANCE_DEGREES = 2.0;
    private static final double TARGET_WRIST_TOLERANCE_DEGREES = 2.0;

    private boolean atTargetPitch() {
        if(adjustedTarget == null) return false;
        return Math.abs(
            MathUtil.angleModulus(inputs.armPitchPosition.getRadians() - adjustedTarget.pitch().getRadians())) < Units
                .degreesToRadians(TARGET_PITCH_TOLERANCE_DEGREES);
    }

    private boolean atTargetHeight() {
        if(adjustedTarget == null) return false;
        return Math
            .abs(inputs.elevatorHeightMeters - adjustedTarget.height().in(Meters)) < TARGET_HEIGHT_TOLERANCE_METERS;
    }

    private boolean atTargetWrist() {
        if(adjustedTarget == null) return false;
        return Math.abs(MathUtil.angleModulus(
            inputs.armWristPosition.getRadians() - adjustedTarget.wristRotation().rotation.getRadians())) < Units
                .degreesToRadians(TARGET_WRIST_TOLERANCE_DEGREES);
    }

    private boolean isAtTarget() {
        return atTargetHeight() && atTargetPitch() && atTargetWrist();
    }

    public void resetToAbsolute() {
        io.resetToAbsolute();
    }

    public void overrideHeightPower(double power) {
        targetState = null;
        io.overrideHeightPower(power, elevatorKg.get());
    }

    public void overridePitchPower(double power) {
        targetState = null;
        io.overridePitchPower(power);
    }

    public void overrideWristPower(double power) {
        targetState = null;
        io.overrideWristPower(power);
    }

    public void overrideEndEffectorPower(double power) {
        targetState = null;
        io.overrideEndEffectorPower(power);
    }

    private ArmState getAdjustedTarget() {
        if(targetState == null) return null;

        WristRotation targetWrist = targetState.wristRotation();
        if(inputs.armPitchPosition.getDegrees() < -90. + 30.) {
            targetWrist = WristRotation.Horizontal;
        }

        EndEffectorState targetEndEffector = targetState.endEffectorState();
        double degreesOff = Math.abs(inputs.armPitchPosition.getDegrees() - targetState.pitch().getDegrees());
        if(targetEndEffector.isHold() && degreesOff > TARGET_PITCH_TOLERANCE_DEGREES * 2) {
            targetEndEffector = EndEffectorState.velocity(-6 * degreesOff / 90.);
        }

        return new ArmState(targetState.pitch(), targetState.height(), targetWrist, targetEndEffector);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        if(targetState != null) {
            adjustedTarget = getAdjustedTarget();

            io.setElevatorHeight(adjustedTarget.height().in(Meters), elevatorKg.get());
            io.setArmPitchPosition(adjustedTarget.pitch(),
                Math.abs(Math.cos(inputs.armPitchPosition.getRadians())) * armPitchKg.get());
            io.setWristRotation(adjustedTarget.wristRotation());
            io.setEndEffectorState(adjustedTarget.endEffectorState());

            Logger.recordOutput("Arm/TargetHeight", adjustedTarget.height().in(Meters));
            Logger.recordOutput("Arm/TargetPitch", adjustedTarget.pitch().getRadians());
            Logger.recordOutput("Arm/TargetWrist", adjustedTarget.wristRotation().rotation.getRadians());
            Logger.recordOutput("Arm/TargetEndEffector",
                adjustedTarget.endEffectorState().getVelocityControl().orElse(0.0));
            Logger.recordOutput("Arm/AtTargetPitch", atTargetPitch());
            Logger.recordOutput("Arm/AtTargetWrist", atTargetWrist());
            Logger.recordOutput("Arm/AtTargetHeight", atTargetHeight());
            Logger.recordOutput("Arm/AtTarget", isAtTarget());
        }

        // TODO: A more robust resetting solution. Our elevator basically never skips, though, so just an operator override is fine.
        // If the elevator is moving slowly, we can reset the encoder position to the elevator height
        // from our sensor.
        // if(inputs.elevatorVelocityMetersPerSecond < 0.03 && inputs.elevatorMotorsConnected
        //     && inputs.validAbsoluteMeasurement) {
        //     io.resetHeight(inputs.absoluteHeightMeters);
        //     Logger.recordOutput("Arm/ElevatorResetting", true);
        // } else {
        //     Logger.recordOutput("Arm/ElevatorResetting", false);
        // }

        visualizer.update(inputs.absoluteHeightMeters, inputs.armPitchPosition, inputs.armWristPosition,
            inputs.gamePiecePresent);

        // Update alerts
        elevatorMotorDisconnectedAlert.set(!inputs.elevatorMotorsConnected);
        elevatorHeightSensorDisconnectedAlert.set(!inputs.elevatorHeightSensorConnected);
        armPitchMotorDisconnectedAlert.set(!inputs.armPitchMotorConnected);
        armWristMotorDisconnectedAlert.set(!inputs.armWristMotorConnected);
        endEffectorMotorDisconnectedAlert.set(!inputs.endEffectorMotorConnected);

        LoggedTracer.record("Arm");
    }
}
