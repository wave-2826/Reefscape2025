package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveConstants.SwerveModuleConfiguration;
import frc.robot.util.AngleAverageFilter;

import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller, and duty cycle
 * absolute encoder.
 */
public class ModuleIOSpark implements ModuleIO {
    private final Rotation2d zeroRotation;

    // Hardware objects
    private final SparkBase driveSpark;
    private final SparkBase turnSpark;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;
    private final SparkAnalogSensor turnAbsoluteEncoder;

    // Closed loop controllers
    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController turnController;

    // Queue inputs from odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

    private final AngleAverageFilter encoderFilter = new AngleAverageFilter(15);
    private boolean didResetToAbsolute = false;

    public ModuleIOSpark(SwerveModuleConfiguration config) {
        zeroRotation = config.zeroOffset();
        driveSpark = new SparkFlex(config.driveMotorCanID(), MotorType.kBrushless);
        turnSpark = new SparkFlex(config.turnMotorCanID(), MotorType.kBrushless);

        // Configure drive motor
        var driveConfig = new SparkFlexConfig();
        driveConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(driveMotorCurrentLimit)
            .voltageCompensation(Constants.voltageCompensation);
        driveConfig.encoder.positionConversionFactor(driveEncoderPositionFactor)
            .velocityConversionFactor(driveEncoderVelocityFactor).uvwMeasurementPeriod(10).uvwAverageDepth(2);
        driveConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(driveKp, 0.0, driveKd, 0.0);
        driveConfig.signals.primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency)).primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20).appliedOutputPeriodMs(20).busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        tryUntilOk(driveSpark, 5,
            () -> driveSpark.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        // Configure turn motor
        var turnConfig = new SparkMaxConfig();
        turnConfig.inverted(turnInverted).idleMode(IdleMode.kBrake).smartCurrentLimit(turnMotorCurrentLimit)
            .voltageCompensation(Constants.voltageCompensation);
        turnConfig.encoder.positionConversionFactor(turnEncoderPositionFactor)
            .velocityConversionFactor(turnEncoderVelocityFactor);
        turnConfig.analogSensor.inverted(turnEncoderInverted)
            .positionConversionFactor(turnAbsoluteEncoderPositionFactor)
            .velocityConversionFactor(turnAbsoluteEncoderPositionFactor);
        // We don't directly use closed loop feedback with the analog sensor because the noise causes motors to
        // jitter regardless of how we tune the PIDs. This can be prevented with the acceptable closed-loop error
        // setting with SmartMotion or MAXMotion, but it doesn't support PID position wrapping :(
        turnConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).positionWrappingEnabled(true)
            .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput).pidf(turnKp, 0.0, turnKd, 0.0);
        // TODO: Compare odometry accuracy
        turnConfig.signals.analogPositionAlwaysOn(true).analogPositionPeriodMs(20).analogVelocityAlwaysOn(true)
            .analogVelocityPeriodMs(20).primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))
            .primaryEncoderVelocityAlwaysOn(true).primaryEncoderVelocityPeriodMs(20).appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

        tryUntilOk(turnSpark, 5,
            () -> turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        driveEncoder = driveSpark.getEncoder();
        turnEncoder = turnSpark.getEncoder();
        turnAbsoluteEncoder = turnSpark.getAnalog();

        driveController = driveSpark.getClosedLoopController();
        turnController = turnSpark.getClosedLoopController();

        tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));

        // Create odometry queues
        timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
        turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);

        resetToAbsolute();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Update drive inputs
        sparkStickyFault = false;
        ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
        ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
        ifOk(driveSpark, new DoubleSupplier[] {
            driveSpark::getAppliedOutput, driveSpark::getBusVoltage
        }, (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
        ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
        inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

        // Update turn inputs
        sparkStickyFault = false;
        ifOk(turnSpark, turnAbsoluteEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
        ifOk(turnSpark, turnAbsoluteEncoder::getPosition,
            (value) -> inputs.absoluteTurnPosition = new Rotation2d(value));
        ifOk(turnSpark, turnAbsoluteEncoder::getPosition,
            (value) -> inputs.offsetAbsoluteTurnPosition = new Rotation2d(value).minus(zeroRotation));
        // Update absolute turn inputs
        inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(!sparkStickyFault);

        sparkStickyFault = false;
        ifOk(turnSpark, new DoubleSupplier[] {
            turnSpark::getAppliedOutput, turnSpark::getBusVoltage
        }, (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
        ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
        ifOk(turnSpark, turnEncoder::getPosition, (value) -> inputs.relativeTurnPosition = new Rotation2d(value));
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

        // Update odometry inputs
        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositions = turnPositionQueue.stream().map(Rotation2d::new).toArray(Rotation2d[]::new);

        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();

        if(!didResetToAbsolute) {
            encoderFilter.reset();
        }
        didResetToAbsolute = false;
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveSpark.setVoltage(output);
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnSpark.setVoltage(output);
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec, double feedforward) {
        driveController.setReference(velocityRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward,
            ArbFFUnits.kVoltage);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        double setpoint = MathUtil.inputModulus(rotation.getRadians(), turnPIDMinInput, turnPIDMaxInput);
        turnController.setReference(setpoint, ControlType.kPosition);
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD) {
        SparkFlexConfig newConfig = new SparkFlexConfig();
        newConfig.closedLoop.pidf(kP, kI, kD, 0.0);
        tryUntilOk(driveSpark, 5,
            () -> driveSpark.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        SparkFlexConfig newConfig = new SparkFlexConfig();
        newConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
        tryUntilOk(driveSpark, 5,
            () -> driveSpark.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        SparkMaxConfig newConfig = new SparkMaxConfig();
        newConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
        tryUntilOk(turnSpark, 5,
            () -> turnSpark.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
    }

    @Override
    public void resetToAbsolute() {
        // Both the turn encoder and turn absolute encoder have conversion factors
        // that represent radians of the output, so this requires no conversion.

        didResetToAbsolute = true;

        turnEncoder.setPosition(MathUtil.inputModulus(
            encoderFilter.calculate(turnAbsoluteEncoder.getPosition() - zeroRotation.getRadians()), turnPIDMinInput,
            turnPIDMaxInput));
    }

    @Override
    public void setTurnPID(double kP, double kI, double kD, double dFilter) {
        SparkFlexConfig newConfig = new SparkFlexConfig();
        if(dFilter > 1.) {
            dFilter = 1;
            DriverStation.reportError("Invalid D filter value! This would have crashed...", null);
        }
        newConfig.closedLoop.pidf(kP, kI, kD, 0.0).dFilter(dFilter);
        tryUntilOk(turnSpark, 5,
            () -> turnSpark.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
    }

    @Override
    public void setDriveCurrentLimit(int limitAmps) {
        SparkFlexConfig newConfig = new SparkFlexConfig();
        newConfig.smartCurrentLimit(limitAmps);
        tryUntilOk(driveSpark, 5,
            () -> driveSpark.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
    }
}
