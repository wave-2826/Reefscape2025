package frc.robot.subsystems.arm;

import static frc.robot.util.SparkUtil.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmState.WristRotation;
import frc.robot.util.SparkUtil;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.ConfigurationFailedException;

public class ArmIOReal implements ArmIO {
    protected SparkMax elevatorHeightMotorLeader;
    protected SparkMax elevatorHeightMotorFollower;
    protected SparkMax armPitchMotor;
    protected SparkMax armWristMotor;
    protected SparkMax endEffectorMotor;

    private SparkClosedLoopController elevatorHeightController;
    private SparkClosedLoopController armPitchController;
    private SparkClosedLoopController armWristController;
    private SparkClosedLoopController endEffectorController;

    protected RelativeEncoder leaderElevatorHeightEncoder;
    private RelativeEncoder followerElevatorHeightEncoder;
    private AbsoluteEncoder armPitchEncoder;
    private AbsoluteEncoder armWristEncoder;
    private RelativeEncoder armWristRelativeEncoder;
    private RelativeEncoder endEffectorEncoder;

    private Debouncer elevatorConnectedDebouncer = new Debouncer(0.25);
    private Debouncer elevatorHeightSensorConnectedDebouncer = new Debouncer(0.25);
    private Debouncer armPitchConnectedDebouncer = new Debouncer(0.25);
    private Debouncer armWristConnectedDebouncer = new Debouncer(0.25);
    private Debouncer endEffectorConnectedDebouncer = new Debouncer(0.25);

    private Alert laserCANNoiseIssue = new Alert("Elevator Laser CAN noise error", AlertType.kWarning);
    private Alert laserCANWeakSignal = new Alert("Elevator Laser CAN weak signal", AlertType.kWarning);
    private Alert laserCANOutOfBounds = new Alert("Elevator Laser CAN out of bounds", AlertType.kWarning);
    private Alert laserCANWraparound = new Alert("Elevator Laser CAN wraparound", AlertType.kWarning);
    private Alert laserCANInvalidHeight = new Alert("Elevator Laser CAN reading invalid height", AlertType.kWarning);

    // This holding logic is in the IO implementation because we don't recreate it in
    // simulation.
    /** If the end effector was in hold mode last update. Used to store the held position. */
    private boolean endEffectorWasHolding = false;
    /** The wrist position when we last started holding. */
    private Rotation2d startHoldWristPosition = Rotation2d.fromDegrees(0.);
    /** The end effector position when we last started holding. */
    private Rotation2d startHoldEndEffectorPosition = Rotation2d.fromDegrees(0.);

    /** The elevator height sensor. Uses an interface to allow for simulation. */
    protected LaserCanInterface elevatorHeightSensor;

    /**
     * If we have reset to absolute. If we can't immediately reset to an absolute position because we have invalid data,
     * we attempt to repeatedly until it works.
     */
    protected boolean needsToReset = false;

    public ArmIOReal() {
        this(null);
    }

    public ArmIOReal(LaserCanInterface overrideLaserCan) {
        // Create motor contorllers 
        elevatorHeightMotorLeader = new SparkMax(ArmConstants.ElevatorConstants.elevatorHeightMotorLeaderId,
            MotorType.kBrushless);
        elevatorHeightMotorFollower = new SparkMax(ArmConstants.ElevatorConstants.elevatorHeightMotorFollowerId,
            MotorType.kBrushless);

        armPitchMotor = new SparkMax(ArmConstants.ShoulderConstants.armPitchMotorId, MotorType.kBrushless);
        armWristMotor = new SparkMax(ArmConstants.ShoulderConstants.armWristMotorId, MotorType.kBrushless);

        endEffectorMotor = new SparkMax(ArmConstants.EndEffectorConstants.endEffectorMotorId, MotorType.kBrushless);

        // Configure motor controllers
        SparkFlexConfig elevatorMotorLeaderConfig = new SparkFlexConfig();
        elevatorMotorLeaderConfig.closedLoop.apply(ArmConstants.ElevatorConstants.elevatorPID.getConfig())
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        elevatorMotorLeaderConfig.encoder
            .positionConversionFactor(ArmConstants.ElevatorConstants.elevatorPositionConversionFactor)
            .velocityConversionFactor(ArmConstants.ElevatorConstants.elevatorVelocityConversionFactor);
        elevatorMotorLeaderConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ArmConstants.ElevatorConstants.elevatorMotorCurrentLimit)
            .voltageCompensation(Constants.voltageCompensation)
            .inverted(ArmConstants.ElevatorConstants.elevatorMotorInverted);
        elevatorMotorLeaderConfig.signals.apply(SparkUtil.defaultSignals).primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderVelocityAlwaysOn(true).primaryEncoderPositionPeriodMs(20).primaryEncoderVelocityPeriodMs(20);

        SparkFlexConfig elevatorMotorFollowerConfig = new SparkFlexConfig();
        elevatorMotorFollowerConfig.encoder
            .positionConversionFactor(ArmConstants.ElevatorConstants.elevatorPositionConversionFactor)
            .velocityConversionFactor(ArmConstants.ElevatorConstants.elevatorVelocityConversionFactor);
        elevatorMotorFollowerConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ArmConstants.ElevatorConstants.elevatorMotorCurrentLimit)
            .voltageCompensation(Constants.voltageCompensation);
        elevatorMotorFollowerConfig.follow(ArmConstants.ElevatorConstants.elevatorHeightMotorLeaderId, true);
        elevatorMotorFollowerConfig.signals.apply(SparkUtil.defaultSignals).primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderVelocityAlwaysOn(true).primaryEncoderPositionPeriodMs(20).primaryEncoderVelocityPeriodMs(20);

        tryUntilOk(elevatorHeightMotorLeader, 5, () -> elevatorHeightMotorLeader.configure(elevatorMotorLeaderConfig,
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(elevatorHeightMotorFollower, 5, () -> elevatorHeightMotorFollower
            .configure(elevatorMotorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        SparkMaxConfig armPitchConfig = new SparkMaxConfig();
        armPitchConfig.closedLoop.apply(ArmConstants.ShoulderConstants.armPitchPID.getConfig())
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder); // No position wrapping -- we shouldn't try to go all the way around.
        armPitchConfig.encoder.positionConversionFactor(ArmConstants.ShoulderConstants.pitchPositionConversionFactor)
            .velocityConversionFactor(ArmConstants.ShoulderConstants.pitchVelocityConversionFactor);
        armPitchConfig.absoluteEncoder
            .positionConversionFactor(ArmConstants.ShoulderConstants.pitchAbsolutePositionFactor)
            .velocityConversionFactor(ArmConstants.ShoulderConstants.pitchAbsoluteVelocityFactor)
            .zeroOffset(Constants.isSim ? 0 : ArmConstants.ShoulderConstants.pitchZeroOffset).zeroCentered(true)
            .inverted(ArmConstants.ShoulderConstants.pitchEncoderInverted);
        armPitchConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ArmConstants.ShoulderConstants.pitchMotorCurrentLimit)
            .voltageCompensation(Constants.voltageCompensation)
            .inverted(ArmConstants.ShoulderConstants.pitchMotorInverted).closedLoopRampRate(0.05);
        armPitchConfig.signals.apply(SparkUtil.defaultSignals) //
            .absoluteEncoderPositionAlwaysOn(true).absoluteEncoderVelocityAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs(20).absoluteEncoderVelocityPeriodMs(20);

        tryUntilOk(armPitchMotor, 5, () -> armPitchMotor.configure(armPitchConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));

        SparkMaxConfig armWristConfig = new SparkMaxConfig();
        armWristConfig.closedLoop.apply(ArmConstants.ShoulderConstants.armWristPID.getConfig())
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder).positionWrappingEnabled(true)
            .positionWrappingInputRange(-Math.PI, Math.PI);
        armWristConfig.encoder.positionConversionFactor(ArmConstants.ShoulderConstants.wristPositionConversionFactor)
            .velocityConversionFactor(ArmConstants.ShoulderConstants.wristVelocityConversionFactor);
        armWristConfig.absoluteEncoder
            .positionConversionFactor(ArmConstants.ShoulderConstants.wristAbsolutePositionFactor)
            .velocityConversionFactor(ArmConstants.ShoulderConstants.wristAbsoluteVelocityFactor)
            .zeroOffset(Constants.isSim ? 0 : ArmConstants.ShoulderConstants.wristZeroOffset).zeroCentered(true);
        armWristConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ArmConstants.ShoulderConstants.wristMotorCurrentLimit)
            .voltageCompensation(Constants.voltageCompensation)
            .inverted(ArmConstants.ShoulderConstants.wristMotorInverted).closedLoopRampRate(0.05);
        // We use the forward and backward limit switch for the vertical and horizontal game piece
        // detectors respectvely. We don't want them to act as actual limit switches.
        armWristConfig.limitSwitch.forwardLimitSwitchEnabled(false).reverseLimitSwitchEnabled(false);
        armWristConfig.signals.apply(SparkUtil.defaultSignals) //
            .limitsPeriodMs(20).absoluteEncoderPositionAlwaysOn(true).absoluteEncoderVelocityAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs(20).absoluteEncoderVelocityPeriodMs(20);

        tryUntilOk(armWristMotor, 5, () -> armWristMotor.configure(armWristConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));

        SparkMaxConfig endEffectorConfig = new SparkMaxConfig();
        endEffectorConfig.closedLoop.apply(ArmConstants.EndEffectorConstants.endEffectorPID.getConfig())
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        endEffectorConfig.encoder
            .positionConversionFactor(ArmConstants.EndEffectorConstants.endEffectorPositionConversionFactor)
            .velocityConversionFactor(ArmConstants.EndEffectorConstants.endEffectorVelocityConversionFactor);
        endEffectorConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ArmConstants.EndEffectorConstants.endEffectorMotorCurrentLimit)
            .voltageCompensation(Constants.voltageCompensation)
            .inverted(ArmConstants.EndEffectorConstants.endEffectorMotorInverted).closedLoopRampRate(0.05);
        endEffectorConfig.signals.apply(SparkUtil.defaultSignals) //
            .primaryEncoderPositionAlwaysOn(true).primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityAlwaysOn(true).primaryEncoderVelocityPeriodMs(20);

        tryUntilOk(endEffectorMotor, 5, () -> endEffectorMotor.configure(endEffectorConfig,
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        // Register the tunable PIDs
        ArmConstants.ElevatorConstants.elevatorPID.configureSparkOnChange(elevatorHeightMotorLeader);
        ArmConstants.ElevatorConstants.elevatorPID.configureSparkOnChange(elevatorHeightMotorFollower);

        ArmConstants.ShoulderConstants.armPitchPID.configureSparkOnChange(armPitchMotor);
        ArmConstants.ShoulderConstants.armWristPID.configureSparkOnChange(armWristMotor);
        ArmConstants.EndEffectorConstants.endEffectorPID.configureSparkOnChange(endEffectorMotor);

        // Register the sparks for fault alerts
        registerSparkFaultAlerts(elevatorHeightMotorLeader, "Elevator height leader");
        registerSparkFaultAlerts(elevatorHeightMotorFollower, "Elevator height follower");
        registerSparkFaultAlerts(armPitchMotor, "Arm pitch motor");
        registerSparkFaultAlerts(armWristMotor, "Arm wrist motor");
        registerSparkFaultAlerts(endEffectorMotor, "End effector motor");

        // Create closed loop controllers
        elevatorHeightController = elevatorHeightMotorLeader.getClosedLoopController();

        armPitchController = armPitchMotor.getClosedLoopController();
        armWristController = armWristMotor.getClosedLoopController();
        endEffectorController = endEffectorMotor.getClosedLoopController();

        leaderElevatorHeightEncoder = elevatorHeightMotorLeader.getEncoder();
        followerElevatorHeightEncoder = elevatorHeightMotorFollower.getEncoder();

        armPitchEncoder = armPitchMotor.getAbsoluteEncoder();
        armWristEncoder = armWristMotor.getAbsoluteEncoder();
        armWristRelativeEncoder = armWristMotor.getEncoder();

        endEffectorEncoder = endEffectorMotor.getEncoder();

        tryUntilOk(elevatorHeightMotorLeader, 2,
            () -> leaderElevatorHeightEncoder.setPosition(ArmConstants.ElevatorConstants.elevatorStartingHeightMeters));
        tryUntilOk(elevatorHeightMotorFollower, 2, () -> followerElevatorHeightEncoder
            .setPosition(ArmConstants.ElevatorConstants.elevatorStartingHeightMeters));

        if(overrideLaserCan != null) {
            elevatorHeightSensor = overrideLaserCan;
        } else {
            elevatorHeightSensor = new LaserCan(ArmConstants.ElevatorConstants.elevatorHeightSensorId);
        }
        try {
            elevatorHeightSensor.setRangingMode(LaserCan.RangingMode.LONG);
            elevatorHeightSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 6, 6));
            elevatorHeightSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_50MS);
        } catch(ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
    }

    /** To minimize latency, this should be called before updateInputs. */
    @Override
    public void resetToAbsolute() {
        needsToReset = true;
    }

    @Override
    public void resetToBottom() {
        tryUntilOk(elevatorHeightMotorLeader, 2,
            () -> leaderElevatorHeightEncoder.setPosition(ArmConstants.ElevatorConstants.bottomResetHeightMeters));
        tryUntilOk(elevatorHeightMotorFollower, 2,
            () -> followerElevatorHeightEncoder.setPosition(ArmConstants.ElevatorConstants.bottomResetHeightMeters));
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        // Update elevator height motor inputs
        sparkStickyFault = false;

        // NOTE: We used to take the average of these two motors. DO NOT DO THIS WITH FOLLOWER MODE!!
        // This was the source of longstanding resetting bugs :(
        ifOk(elevatorHeightMotorLeader, leaderElevatorHeightEncoder::getPosition,
            (v) -> inputs.elevatorHeightMeters = v);
        ifOk(elevatorHeightMotorLeader, leaderElevatorHeightEncoder::getVelocity,
            (v) -> inputs.elevatorVelocityMetersPerSecond = v);
        inputs.elevatorMotorsConnected = elevatorConnectedDebouncer.calculate(!sparkStickyFault);

        Measurement measurement = elevatorHeightSensor.getMeasurement();
        inputs.elevatorHeightSensorConnected = elevatorHeightSensorConnectedDebouncer.calculate(measurement != null);
        if(measurement != null) {
            inputs.absoluteHeightMeters = (float) measurement.distance_mm / 1000.;
            boolean invalidHeight = inputs.absoluteHeightMeters < ArmConstants.ElevatorConstants.bottomResetHeightMeters;
            inputs.validAbsoluteMeasurement = measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT
                && !invalidHeight;

            Logger.recordOutput("Arm/ElevatorNeedsToReset", needsToReset);

            // For some reason, this doens't work with simulated sparks. We do this in ArmIOSim instead.
            if(needsToReset && inputs.validAbsoluteMeasurement && Constants.currentMode != Constants.Mode.SIM) {
                // TODO: Use ifOk and only adjust once this works?
                tryUntilOk(elevatorHeightMotorLeader, 2,
                    () -> leaderElevatorHeightEncoder.setPosition(inputs.absoluteHeightMeters));
                tryUntilOk(elevatorHeightMotorFollower, 2,
                    () -> followerElevatorHeightEncoder.setPosition(inputs.absoluteHeightMeters));
                needsToReset = false;
            }

            laserCANNoiseIssue.set(measurement.status == LaserCan.LASERCAN_STATUS_NOISE_ISSUE);
            laserCANWeakSignal.set(measurement.status == LaserCan.LASERCAN_STATUS_WEAK_SIGNAL);
            laserCANOutOfBounds.set(measurement.status == LaserCan.LASERCAN_STATUS_OUT_OF_BOUNDS);
            laserCANWraparound.set(measurement.status == LaserCan.LASERCAN_STATUS_WRAPAROUND);
            laserCANInvalidHeight.set(invalidHeight);

            Logger.recordOutput("Arm/ElevatorAmbientLight", measurement.ambient);
        }

        // Update arm pitch motor inputs
        sparkStickyFault = false;
        ifOk(armPitchMotor, armPitchEncoder::getPosition, (v) -> inputs.armPitchPosition = Rotation2d.fromRadians(v));
        ifOk(armPitchMotor, armPitchEncoder::getVelocity, (v) -> inputs.armPitchVelocity = v);
        inputs.armPitchMotorConnected = armPitchConnectedDebouncer.calculate(!sparkStickyFault);

        // Update arm wrist motor inputs
        sparkStickyFault = false;
        ifOk(armWristMotor, armWristEncoder::getPosition, (v) -> inputs.armWristPosition = Rotation2d.fromRadians(v));
        ifOk(armWristMotor, armWristEncoder::getVelocity, (v) -> inputs.armWristVelocity = v);

        // TODO: Update once we have game piece sensors (if we ever will...)
        inputs.gamePiecePresent = false;

        inputs.armWristMotorConnected = armWristConnectedDebouncer.calculate(!sparkStickyFault);

        // Update end effector motor inputs
        sparkStickyFault = false;
        ifOk(endEffectorMotor, endEffectorEncoder::getVelocity, (v) -> inputs.endEffectorVelocity = v);
        inputs.endEffectorMotorConnected = endEffectorConnectedDebouncer.calculate(!sparkStickyFault);
    }

    @Override
    public void setElevatorHeight(double heightMeters, double feedforwardVolts) {
        elevatorHeightController.setReference(heightMeters, ControlType.kPosition, ClosedLoopSlot.kSlot0,
            feedforwardVolts, ArbFFUnits.kVoltage);
    }

    @Override
    public void setArmPitchPosition(Rotation2d position, double feedforward) {
        armPitchController.setReference(MathUtil.angleModulus(position.getRadians()), ControlType.kPosition,
            ClosedLoopSlot.kSlot0, feedforward, ArbFFUnits.kVoltage);
    }

    @Override
    public void setWristRotation(WristRotation rotation) {
        armWristController.setReference(MathUtil.angleModulus(-rotation.rotation.getRadians()), ControlType.kPosition,
            ArmConstants.ShoulderConstants.armWristPositionSlot);
    }

    public void overrideHeightPower(double power, double feedforward) {
        elevatorHeightMotorLeader.setVoltage(power * RobotController.getBatteryVoltage() + feedforward);
    }

    public void overridePitchPower(double power) {
        armPitchController.setReference(power, ControlType.kDutyCycle);
    }

    public void overrideWristPower(double power) {
        armWristController.setReference(power, ControlType.kDutyCycle);
    }

    public void overrideEndEffectorPower(double power) {
        endEffectorController.setReference(power, ControlType.kDutyCycle);
    }

    @Override
    public void setEndEffectorState(EndEffectorState state) {
        // Positive velocity is outward.
        var velocity = state.getVelocityControl();
        if(velocity.isPresent()) {
            endEffectorController.setReference(velocity.getAsDouble(), ControlType.kVelocity,
                ArmConstants.EndEffectorConstants.endEffectorVelocitySlot);
            endEffectorWasHolding = false;
        } else {
            // Holding mode
            if(!endEffectorWasHolding) {
                // Store the current wrist and end effector positions
                startHoldWristPosition = Rotation2d.fromRadians(armWristRelativeEncoder.getPosition());
                startHoldEndEffectorPosition = Rotation2d.fromRadians(endEffectorEncoder.getPosition());
            }
            endEffectorWasHolding = true;

            // We don't simulate the coaxial coupling in simulation, so we turn it off.
            double couplingFactor = Constants.isSim ? 0 : ArmConstants.EndEffectorConstants.endEffectorCouplingFactor;
            double holdPositionRad = startHoldEndEffectorPosition.getRadians()
                - (armWristRelativeEncoder.getPosition() - startHoldWristPosition.getRadians()) * couplingFactor;
            endEffectorController.setReference(holdPositionRad, ControlType.kPosition,
                ArmConstants.EndEffectorConstants.endEffectorPositionSlot);
        }
    }
}
