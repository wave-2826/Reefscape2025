package frc.robot.subsystems.intake;

import java.util.HashMap;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Controls;
import frc.robot.util.LoggedTracer;

/**
 * The intake subsystem. Manages all intake motors.
 */
public class Intake extends SubsystemBase {
    private static final double INTAKE_PITCH_TOLERANCE_DEGREES = 6.0;

    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private IntakeVisualizer visualizer = new IntakeVisualizer("intake");

    public enum IntakeState {
        // @formatter:off
        IntakeDown(Rotation2d.kZero, 1.0, true),
        OuttakeDown(Rotation2d.kZero, -1.0, true),
        Up(Rotation2d.fromDegrees(80.0), 0.0, false);
        // @formatter:on

        public final Rotation2d pitch;
        public final double speed;
        /** Whether the intake should be set to coast once it's near its pitch setpoint. */
        public final boolean coast;

        IntakeState(Rotation2d pitch, double speed, boolean coast) {
            this.pitch = pitch;
            this.speed = speed;
            this.coast = coast;
        }
    }

    public enum TransportTarget {
        NoPiece(0.0), MovingPiece(1.0), PieceWaitingForArm(0.0);

        public final double speed;

        TransportTarget(double speed) {
            this.speed = speed;
        }
    }

    private IntakeState targetIntakeState = IntakeState.Up;
    private boolean usingClosedLoopControl = false;

    /**
     * A debouncer for the middle sensor.
     */
    private Debouncer middleDebouncer = new Debouncer(0.0, DebounceType.kFalling);

    /**
     * A map from transport sensor states to the current target. The key is represented as binary in the form 0b(intake
     * sensor, middle sensor, end sensor)
     */
    private static HashMap<Integer, TransportTarget> transportTargetMap = new HashMap<>();
    static {
        transportTargetMap.put(0b000, TransportTarget.NoPiece);
        transportTargetMap.put(0b001, TransportTarget.PieceWaitingForArm);
        transportTargetMap.put(0b010, TransportTarget.MovingPiece);
        transportTargetMap.put(0b011, TransportTarget.MovingPiece);
        transportTargetMap.put(0b100, TransportTarget.MovingPiece);
        transportTargetMap.put(0b101, TransportTarget.MovingPiece); // Invalid state; maybe we should eventually outtake here?
        transportTargetMap.put(0b110, TransportTarget.MovingPiece);
        transportTargetMap.put(0b111, TransportTarget.MovingPiece); // Invalid state; maybe we should eventually outtake here?
    }

    private final Alert transportMotorDisconnectedAlert = new Alert("Transport motor disconnected!", AlertType.kError);
    private final Alert intakePowerMotorDisconnectedAlert = new Alert("Intake power motor disconnected!",
        AlertType.kError);
    private final Alert intakePitchMotorDisconnectedAlert = new Alert("Intake pitch motor disconnected!",
        AlertType.kError);
    private final Alert intakePitchEncoderDisconnectedAlert = new Alert("Intake pitch encoder reading 0!",
        AlertType.kWarning);

    /**
     * The mappings from intake state sensors to our target state.
     */
    public Intake(IntakeIO io) {
        this.io = io;

        new Trigger(() -> inputs.intakeSensorTriggered).onTrue(
            Controls.getInstance().controllerRumbleWhileRunning(true, true, RumbleType.kBothRumble).withTimeout(0.1)
                .andThen(Commands.waitSeconds(0.05)).repeatedly().withTimeout(0.4).onlyIf(DriverStation::isTeleop));
    }

    private TransportTarget getTransportTarget(boolean useDebounce) {
        int key = 0b000;
        if(inputs.intakeSensorTriggered) key |= 0b100;
        if(useDebounce ? middleDebouncer.calculate(inputs.middleSensorTriggered)
            : inputs.middleSensorTriggered) key |= 0b010;
        if(inputs.endSensorTriggered) key |= 0b001;

        return transportTargetMap.get(key);
    }

    public Command reset() {
        return runOnce(() -> {
            usingClosedLoopControl = true;
            targetIntakeState = IntakeState.Up;
        });
    }

    @AutoLogOutput(key = "Intake/AtPitchSetpoint")
    private boolean atPitchSetpoint() {
        return Math.abs(
            inputs.intakePitch.getDegrees() - targetIntakeState.pitch.getDegrees()) < INTAKE_PITCH_TOLERANCE_DEGREES;
    }

    /**
     * Sets the target state of the intake.
     * @param power The desired power for the intake, from 0 to 1.
     */
    public void overrideIntakeSpeed(double power) {
        usingClosedLoopControl = false;
        io.runVelocity(power, power);
    }

    public void overridePitchPower(double power) {
        usingClosedLoopControl = false;
        io.overridePitchPower(power);
    }

    /**
     * Sets the intake to a specific state.
     */
    public void setIntakeState(IntakeState state) {
        targetIntakeState = state;
        usingClosedLoopControl = true;
    }

    public boolean intakeSensorTriggered() {
        return inputs.intakeSensorTriggered;
    }

    public boolean pieceInTransport() {
        return inputs.intakeSensorTriggered || inputs.middleSensorTriggered;
    }

    public boolean pieceWaitingForArm() {
        return getTransportTarget(false) == TransportTarget.PieceWaitingForArm;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        TransportTarget transportTarget = getTransportTarget(true);

        if(usingClosedLoopControl) {
            if(Math.abs(
                inputs.intakePitch.getDegrees() - targetIntakeState.pitch.getDegrees()) < INTAKE_PITCH_TOLERANCE_DEGREES
                && targetIntakeState.coast) {
                io.setIntakeCoast();
            } else {
                io.setIntakePitch(targetIntakeState.pitch);
            }

            if(IntakeConstants.disableTransportSensors) io.runVelocity(targetIntakeState.speed,
                targetIntakeState.speed);
            else io.runVelocity(targetIntakeState.speed,
                targetIntakeState.speed == 0 ? transportTarget.speed : targetIntakeState.speed);
        }

        transportMotorDisconnectedAlert.set(!inputs.transportMotorConnected);
        intakePowerMotorDisconnectedAlert.set(!inputs.intakePowerMotorConnected);
        intakePitchMotorDisconnectedAlert.set(!inputs.intakePitchMotorConnected);
        intakePitchEncoderDisconnectedAlert.set(inputs.intakePitch.getRadians() == 0);

        visualizer.update(inputs.intakePitch);

        Logger.recordOutput("Intake/TargetPitch", targetIntakeState.pitch);
        Logger.recordOutput("Intake/TargetSpeed", targetIntakeState.speed);
        Logger.recordOutput("Intake/TargetTransportSpeed", transportTarget.speed);

        LoggedTracer.record("Intake");
    }
}
