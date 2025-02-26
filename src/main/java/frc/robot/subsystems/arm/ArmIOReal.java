package frc.robot.subsystems.arm;

import static frc.robot.util.SparkUtil.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
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
    protected SparkFlex endEffectorMotor;

    private SparkClosedLoopController elevatorHeightController;
    private SparkClosedLoopController armPitchController;
    private SparkClosedLoopController armWristController;
    private SparkClosedLoopController endEffectorController;

    private RelativeEncoder elevatorHeightEncoder1;
    private RelativeEncoder elevatorHeightEncoder2;
    private AbsoluteEncoder armPitchEncoder;
    private AbsoluteEncoder armWristEncoder;
    private RelativeEncoder armWristRelativeEncoder;
    private RelativeEncoder endEffectorEncoder;

    private Debouncer elevatorConnectedDebouncer = new Debouncer(0.25);
    private Debouncer elevatorHeightSensorConnectedDebouncer = new Debouncer(0.25);
    private Debouncer armPitchConnectedDebouncer = new Debouncer(0.25);
    private Debouncer armWristConnectedDebouncer = new Debouncer(0.25);
    private Debouncer endEffectorConnectedDebouncer = new Debouncer(0.25);

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

    public ArmIOReal() {
        this(null);
    }

    public ArmIOReal(LaserCanInterface overrideLaserCan) {
        // Create motor contorllers 
        elevatorHeightMotorLeader = new SparkMax(ArmConstants.ElevatorConstants.elevatorHeightMotor1Id,
            MotorType.kBrushless);
        elevatorHeightMotorFollower = new SparkMax(ArmConstants.ElevatorConstants.elevatorHeightMotor2Id,
            MotorType.kBrushless);

        armPitchMotor = new SparkMax(ArmConstants.ShoulderConstants.armPitchMotorId, MotorType.kBrushless);
        armWristMotor = new SparkMax(ArmConstants.ShoulderConstants.armWristMotorId, MotorType.kBrushless);

        endEffectorMotor = new SparkFlex(ArmConstants.EndEffectorConstants.endEffectorMotorId, MotorType.kBrushless);

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
        elevatorMotorFollowerConfig.follow(ArmConstants.ElevatorConstants.elevatorHeightMotor1Id, true);
        elevatorMotorFollowerConfig.signals.apply(SparkUtil.defaultSignals).primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderVelocityAlwaysOn(true).primaryEncoderPositionPeriodMs(20).primaryEncoderVelocityPeriodMs(20);

        tryUntilOk(elevatorHeightMotorLeader, 5, () -> elevatorHeightMotorLeader.configure(elevatorMotorLeaderConfig,
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(elevatorHeightMotorFollower, 5, () -> elevatorHeightMotorFollower
            .configure(elevatorMotorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        SparkMaxConfig armPitchConfig = new SparkMaxConfig();
        armPitchConfig.closedLoop.apply(ArmConstants.ShoulderConstants.armPitchPID.getConfig())
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder).positionWrappingEnabled(true)
            .positionWrappingInputRange(-Math.PI, Math.PI);
        armPitchConfig.encoder.positionConversionFactor(ArmConstants.ShoulderConstants.pitchPositionConversionFactor)
            .velocityConversionFactor(ArmConstants.ShoulderConstants.pitchVelocityConversionFactor);
        armPitchConfig.absoluteEncoder
            .positionConversionFactor(ArmConstants.ShoulderConstants.pitchAbsolutePositionFactor)
            .velocityConversionFactor(ArmConstants.ShoulderConstants.pitchAbsoluteVelocityFactor)
            .zeroOffset(ArmConstants.ShoulderConstants.pitchZeroOffset).zeroCentered(true)
            .inverted(ArmConstants.ShoulderConstants.pitchEncoderInverted);
        armPitchConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ArmConstants.ShoulderConstants.pitchMotorCurrentLimit)
            .voltageCompensation(Constants.voltageCompensation)
            .inverted(ArmConstants.ShoulderConstants.pitchMotorInverted).closedLoopRampRate(0.5);
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
            .zeroOffset(ArmConstants.ShoulderConstants.wristZeroOffset).zeroCentered(true);
        armWristConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ArmConstants.ShoulderConstants.wristMotorCurrentLimit)
            .voltageCompensation(Constants.voltageCompensation)
            .inverted(ArmConstants.ShoulderConstants.wristMotorInverted).closedLoopRampRate(0.5);
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
            .inverted(ArmConstants.EndEffectorConstants.endEffectorMotorInverted).closedLoopRampRate(0.5);
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

        // Create closed loop controllers
        elevatorHeightController = elevatorHeightMotorLeader.getClosedLoopController();

        armPitchController = armPitchMotor.getClosedLoopController();
        armWristController = armWristMotor.getClosedLoopController();
        endEffectorController = endEffectorMotor.getClosedLoopController();

        elevatorHeightEncoder1 = elevatorHeightMotorLeader.getEncoder();
        elevatorHeightEncoder2 = elevatorHeightMotorFollower.getEncoder();

        armPitchEncoder = armPitchMotor.getAbsoluteEncoder();
        armWristEncoder = armWristMotor.getAbsoluteEncoder();
        armWristRelativeEncoder = armWristMotor.getEncoder();

        endEffectorEncoder = endEffectorMotor.getEncoder();

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

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        // Update elevator height motor inputs
        sparkStickyFault = false;

        // We average the two encoders' readings because they may not be perfectly in sync.
        ifOk(new SparkBase[] {
            elevatorHeightMotorLeader, elevatorHeightMotorFollower
        }, new DoubleSupplier[] {
            elevatorHeightEncoder1::getPosition, elevatorHeightEncoder2::getPosition
        }, (v) -> inputs.elevatorHeightMeters = (v[0] + v[1]) / 2.);
        ifOk(new SparkBase[] {
            elevatorHeightMotorLeader, elevatorHeightMotorFollower
        }, new DoubleSupplier[] {
            elevatorHeightEncoder1::getVelocity, elevatorHeightEncoder2::getVelocity
        }, (v) -> inputs.elevatorVelocityMetersPerSecond = (v[0] + v[1]) / 2.);
        inputs.elevatorMotorsConnected = elevatorConnectedDebouncer.calculate(!sparkStickyFault);

        // If the elevator is moving slowly, we can reset the encoder position to the elevator height
        // from our sensor.
        Measurement measurement = elevatorHeightSensor.getMeasurement();
        inputs.elevatorHeightSensorConnected = elevatorHeightSensorConnectedDebouncer.calculate(measurement != null);
        if(inputs.elevatorVelocityMetersPerSecond < 0.03 && inputs.elevatorMotorsConnected && measurement != null
            && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            double position = (double) (measurement.distance_mm) / 1000.;
            // elevatorHeightEncoder1.setPosition(position);
            // elevatorHeightEncoder2.setPosition(position);
            // This is hacky
            // TODO: Wtf
            Logger.recordOutput("Arm/ElevatorResetting", true);
        } else {
            Logger.recordOutput("Arm/ElevatorResetting", false);
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
        // TODO: Update once we have game piece sensors
        inputs.gamePiecePresent = false;

        inputs.armWristMotorConnected = armWristConnectedDebouncer.calculate(!sparkStickyFault);

        // Update end effector motor inputs
        sparkStickyFault = false;
        ifOk(endEffectorMotor, endEffectorEncoder::getVelocity, (v) -> inputs.endEffectorVelocity = v);
        inputs.endEffectorMotorConnected = endEffectorConnectedDebouncer.calculate(!sparkStickyFault);
    }

    @Override
    public void setElevatorHeight(double heightMeters) {
        elevatorHeightController.setReference(heightMeters, ControlType.kPosition, ClosedLoopSlot.kSlot0,
            ArmConstants.ElevatorConstants.elevatorKg, ArbFFUnits.kVoltage);
    }

    @Override
    public void setArmPitchPosition(Rotation2d position, double feedforward) {
        armPitchController.setReference(-MathUtil.angleModulus(position.getRadians()), ControlType.kPosition,
            ClosedLoopSlot.kSlot0, feedforward, ArbFFUnits.kVoltage);
    }

    @Override
    public void setWristRotation(WristRotation rotation) {
        armWristController.setReference(MathUtil.angleModulus(rotation.rotation.getRadians()), ControlType.kPosition,
            ArmConstants.ShoulderConstants.armWristPositionSlot);
    }

    @Override
    public void setEndEffectorState(EndEffectorState state) {
        var velocity = state.getVelocityControl();
        if(velocity.isPresent()) {
            endEffectorController.setReference(velocity.get(), ControlType.kVelocity,
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
            double couplingFactor = Constants.currentMode == Constants.Mode.SIM ? 0
                : ArmConstants.EndEffectorConstants.endEffectorCouplingFactor;
            double holdPositionRad = startHoldEndEffectorPosition.getRadians()
                - (armWristRelativeEncoder.getPosition() - startHoldWristPosition.getRadians()) * couplingFactor;
            endEffectorController.setReference(holdPositionRad, ControlType.kPosition,
                ArmConstants.EndEffectorConstants.endEffectorPositionSlot);
        }
    }
}
