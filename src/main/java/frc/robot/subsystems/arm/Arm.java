package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Meters;

import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.robot.RobotState;
import frc.robot.subsystems.arm.ArmState.WristRotation;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SuppliedWait;
import frc.robot.commands.LoggedCommand;

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

    private final Alert armPitchEncoderDisconnectedAlert = new Alert("Arm pitch encoder reading 0!",
        AlertType.kWarning);
    private final Alert armWristEncoderDisconnectedAlert = new Alert("Arm wrist encoder reading 0!",
        AlertType.kWarning);

    private static final LoggedTunableNumber elevatorKg = new LoggedTunableNumber("Arm/elevatorKg");
    private static final LoggedTunableNumber armPitchKg = new LoggedTunableNumber("Arm/pitchKg");

    private static final LoggedTunableNumber armResetTolerance = new LoggedTunableNumber(// Inches
        "Arm/Reset/Tolerance", 0.25);
    private static final LoggedTunableNumber armResetTimeout = new LoggedTunableNumber(// Seconds
        "Arm/Reset/Timeout", 0.5);

    public static boolean resetWithAbsoluteSensorEnabled = true;
    private final Alert resetWithAbsoluteSensorOffAlert = new Alert("Elevator reset with LaserCAN turned off!",
        AlertType.kInfo);

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

    public Command goToStateCommand(ArmState state, double timeoutSeconds) {
        return this.run(() -> {
            targetState = state;
            adjustedTarget = getAdjustedTarget();
        }).until(this::isAtTarget).withName("ArmGoToState").withTimeout(timeoutSeconds);
    }

    public Command goToStateCommand(ArmState state) {
        return goToStateCommand(state, 1.0);
    }

    public Command goToStateCommand(Supplier<ArmState> state) {
        return Commands.defer(() -> goToStateCommand(state.get()), Set.of(this));
    }

    public void setTargetState(ArmState state) {
        targetState = state;
    }

    public Command setTargetStateCommand(Supplier<ArmState> state) {
        return this.run(() -> {
            targetState = state.get();
        });
    }

    public Command waitForCorrectAbsolute() {
        if(!resetWithAbsoluteSensorEnabled) return Commands.none();

        // @formatter:off
        return new LoggedCommand("WaitForCorrect", Commands.sequence(
            new SuppliedWait(armResetTimeout::get),
            Commands.runOnce(this::resetToAbsolute)
        )).until(this::correctAbsoluteMeasurement).andThen(
            new LoggedCommand("WaitForTargetAfterReset",
                new SuppliedWait(armResetTimeout::get).until(this::atTargetHeight)
            )
        );
        // @formatter:on
    }

    private boolean correctAbsoluteMeasurement() {
        return Math.abs(inputs.absoluteHeightMeters - inputs.elevatorHeightMeters) < Units
            .inchesToMeters(armResetTolerance.get()) && inputs.validAbsoluteMeasurement;
    }

    private static final double TARGET_HEIGHT_TOLERANCE_METERS = Units.inchesToMeters(1.5);
    private static final double TARGET_PITCH_TOLERANCE_DEGREES = 8.0;
    private static final double TARGET_WRIST_TOLERANCE_DEGREES = 5.0;

    public ArmState getCurrentTargetState() {
        return adjustedTarget;
    }

    private boolean atTargetPitch() {
        if(adjustedTarget == null) return true;
        return Math.abs(
            MathUtil.angleModulus(inputs.armPitchPosition.getRadians() - adjustedTarget.pitch().getRadians())) < Units
                .degreesToRadians(TARGET_PITCH_TOLERANCE_DEGREES);
    }

    private boolean atTargetHeight() {
        if(adjustedTarget == null) return true;
        return Math
            .abs(inputs.elevatorHeightMeters - adjustedTarget.height().in(Meters)) < TARGET_HEIGHT_TOLERANCE_METERS;
    }

    private boolean atTargetWrist() {
        if(adjustedTarget == null) return true;
        return Math.abs(MathUtil.angleModulus(
            inputs.armWristPosition.getRadians() - adjustedTarget.wristRotation().rotation.getRadians())) < Units
                .degreesToRadians(TARGET_WRIST_TOLERANCE_DEGREES);
    }

    private boolean isAtTarget() {
        return atTargetHeight() && atTargetPitch() && atTargetWrist();
    }

    public void resetToAbsolute() {
        if(!resetWithAbsoluteSensorEnabled) return;

        if(!inputs.validAbsoluteMeasurement) {
            if(DriverStation.isTeleop()) {
                Controls.getInstance().controllerRumbleWhileRunning(false, true, RumbleType.kBothRumble)
                    .withTimeout(0.2).andThen(Commands.waitSeconds(0.1)).repeatedly().withTimeout(1).schedule();
            }
        } else {
            io.resetToAbsolute();
        }
    }

    public void resetToBottom() {
        io.resetToBottom();
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

    /**
     * Gets the current elevator height as a percentage of the maximum height. This is used to scale our maximum allowed
     * acceleration.
     * @return
     */
    public double getElevatorHeightPercent() {
        return MathUtil.clamp((inputs.elevatorHeightMeters - ArmConstants.ElevatorConstants.bottomResetHeightMeters)
            / (ArmConstants.ElevatorConstants.maxElevatorHeight.in(Meters)
                - ArmConstants.ElevatorConstants.bottomResetHeightMeters),
            0.0, 1.0);
    }

    /**
     * Gets whether reef lineup is currently safe. If this returns true, the arm is in a state that will allow us to
     * line up close to the reef. If false, the arm might hit the reef.
     * @return
     */
    public boolean getReefLineupSafe() {
        if(adjustedTarget == null) return true;

        double lookaheadSecs = 0.4;
        Rotation2d lookaheadArmPosition = inputs.armPitchPosition
            .plus(Rotation2d.fromRadians(inputs.armPitchVelocity * lookaheadSecs));

        // If the current and target arm pitch is less than the given threshold, we aren't safe to line up
        double lowDangerPitch = -90 + 30;
        if(adjustedTarget.pitch().getDegrees() < lowDangerPitch && lookaheadArmPosition.getDegrees() < lowDangerPitch) {
            return false;
        }

        // If the current target arm pitch is above the given threshold, we are safe to line up
        double highSafePitch = 20;
        if(adjustedTarget.pitch().getDegrees() > highSafePitch && lookaheadArmPosition.getDegrees() > highSafePitch) {
            return true;
        }

        // If the elevator and arm are relatively close to their setpoints, we trust ourself
        if(Math.abs(inputs.elevatorHeightMeters - adjustedTarget.height().in(Meters)) < Units.inchesToMeters(4)
            && Math.abs(lookaheadArmPosition.getRadians() - adjustedTarget.pitch().getRadians()) < Units
                .degreesToRadians(10)) {
            return true;
        }

        return false;
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
            targetEndEffector = EndEffectorState.velocity(-8 * degreesOff / 90.);
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

        visualizer.update(inputs.absoluteHeightMeters, inputs.armPitchPosition, inputs.armWristPosition,
            inputs.gamePiecePresent);

        RobotState.getInstance().updateElevatorHeightPercent(getElevatorHeightPercent());
        RobotState.getInstance().setReefLineupSafe(getReefLineupSafe());

        // Update alerts
        elevatorMotorDisconnectedAlert.set(!inputs.elevatorMotorsConnected);
        elevatorHeightSensorDisconnectedAlert.set(!inputs.elevatorHeightSensorConnected);
        armPitchMotorDisconnectedAlert.set(!inputs.armPitchMotorConnected);
        armWristMotorDisconnectedAlert.set(!inputs.armWristMotorConnected);
        endEffectorMotorDisconnectedAlert.set(!inputs.endEffectorMotorConnected);

        resetWithAbsoluteSensorOffAlert.set(!resetWithAbsoluteSensorEnabled);

        armPitchEncoderDisconnectedAlert.set(inputs.armPitchPosition.getRadians() == 0);
        armWristEncoderDisconnectedAlert.set(inputs.armWristPosition.getRadians() == 0);

        LoggedTracer.record("Arm");
    }
}
