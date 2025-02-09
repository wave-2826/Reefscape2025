package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

public class Module {
    private static final LoggedTunableNumber driveP = new LoggedTunableNumber("Drive/DriveP");
    private static final LoggedTunableNumber driveD = new LoggedTunableNumber("Drive/DriveD");

    private static final LoggedTunableNumber driveS = new LoggedTunableNumber("Drive/DriveS");
    private static final LoggedTunableNumber driveV = new LoggedTunableNumber("Drive/DriveV");
    private static final LoggedTunableNumber driveT = new LoggedTunableNumber("Drive/DriveT");
    private static final LoggedTunableNumber driveA = new LoggedTunableNumber("Drive/DriveA");

    private static final LoggedTunableNumber turnP = new LoggedTunableNumber("Drive/TurnP");
    private static final LoggedTunableNumber turnD = new LoggedTunableNumber("Drive/TurnD");

    private static final LoggedTunableNumber turnDerivativeFilter = new LoggedTunableNumber("Drive/TurnDFilter");

    static {
        driveP.initDefault(DriveConstants.driveKp);
        driveD.initDefault(DriveConstants.driveKd);

        driveS.initDefault(DriveConstants.driveKs);
        driveV.initDefault(DriveConstants.driveKv);
        driveT.initDefault(DriveConstants.driveMotorReduction / DCMotor.getNeoVortex(1).KtNMPerAmp);
        driveA.initDefault(DriveConstants.driveKa);

        turnP.initDefault(DriveConstants.turnKp);
        turnD.initDefault(DriveConstants.turnKd);

        turnDerivativeFilter.initDefault(DriveConstants.turnDerivativeFilter);
    }

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final String name;

    private SimpleMotorFeedforward ffModel;

    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;
    private final Alert turnEncoderDisconnectedAlert;

    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    /**
     * A debouncer used to reset the module to absolute after it hasn't rotated for a period of time. Used to prevent
     * latency-related issues.
     */
    private final Debouncer noMovementDebouncer = new Debouncer(0.25);
    /**
     * The speed considered unmoving for resetting the encoder, in radians per second.
     */
    private static final double SPEED_CONSIDERED_UNMOVING = 0.015;

    public Module(ModuleIO io, String name) {
        this.io = io;
        this.name = name;

        ffModel = new SimpleMotorFeedforward(driveS.get(), driveV.get(), driveA.get());

        driveDisconnectedAlert = new Alert("Disconnected drive motor on " + name + "module", AlertType.kError);
        turnDisconnectedAlert = new Alert("Disconnected turn motor on " + name + "module.", AlertType.kError);
        turnEncoderDisconnectedAlert = new Alert("Disconnected turn encoder on " + name + "module.", AlertType.kError);
    }

    public void periodic() {
        if(driveS.hasChanged(hashCode()) || driveV.hasChanged(hashCode()) || driveA.hasChanged(hashCode())) {
            ffModel = new SimpleMotorFeedforward(driveS.get(), driveV.get(), driveA.get());
        }
        if(driveP.hasChanged(hashCode()) || driveD.hasChanged(hashCode())) {
            io.setDrivePID(driveP.get(), 0, driveD.get());
        }
        if(turnP.hasChanged(hashCode()) || turnD.hasChanged(hashCode())
            || turnDerivativeFilter.hasChanged(hashCode())) {
            io.setTurnPID(turnP.get(), 0, turnD.get(), turnDerivativeFilter.get());
        }

        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + name, inputs);

        // Reset to absolute if we've been unmoving for a relatively long time
        if(noMovementDebouncer.calculate(Math.abs(inputs.turnVelocityRadPerSec) < SPEED_CONSIDERED_UNMOVING)) {
            io.resetToAbsolute();
        }

        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for(int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRad[i] * wheelRadiusMeters;
            Rotation2d angle = inputs.odometryTurnPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        // Update alerts
        driveDisconnectedAlert.set(!inputs.driveConnected);
        turnDisconnectedAlert.set(!inputs.turnConnected);
        turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
    }

    /** Runs the module with the specified setpoint state. */
    public void runSetpoint(SwerveModuleState state) {
        // Optimize velocity setpoint
        state.optimize(getAngle());
        state.cosineScale(inputs.relativeTurnPosition);

        // Apply setpoints
        double speedRadPerSec = state.speedMetersPerSecond / DriveConstants.wheelRadiusMeters;
        io.setDriveVelocity(speedRadPerSec, ffModel.calculate(speedRadPerSec)); // TODO: Acceleration feedforward   
        io.setTurnPosition(state.angle);
    }

    /**
     * Runs the module with the specified setpoint state and a setpoint wheel force used for torque-based feedforward.
     */
    public void runSetpoint(SwerveModuleState state, double wheelTorqueNm) {
        // Optimize velocity setpoint
        state.optimize(getAngle());
        state.cosineScale(inputs.relativeTurnPosition);

        // Apply setpoints
        double speedRadPerSec = state.speedMetersPerSecond / DriveConstants.wheelRadiusMeters;
        io.setDriveVelocity(speedRadPerSec, ffModel.calculate(speedRadPerSec) + wheelTorqueNm * driveT.get());
        io.setTurnPosition(state.angle);
    }

    /**
     * Runs the module with the specified output while controlling to zero degrees.
     */
    public void runCharacterization(double output) {
        io.setDriveOpenLoop(output);
        io.setTurnPosition(new Rotation2d());
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.setDriveOpenLoop(0.0);
        io.setTurnOpenLoop(0.0);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return inputs.relativeTurnPosition;
    }

    /** Returns the current drive position of the module in meters. */
    public double getPositionMeters() {
        return inputs.drivePositionRad * wheelRadiusMeters;
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * wheelRadiusMeters;
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /** Returns the module positions received this cycle. */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    /** Returns the timestamps of the samples received this cycle. */
    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    /** Returns the module position in radians. */
    public double getWheelRadiusCharacterizationPosition() {
        return inputs.drivePositionRad;
    }

    /** Returns the module velocity in rad/sec. */
    public double getFFCharacterizationVelocity() {
        return inputs.driveVelocityRadPerSec;
    }
}
