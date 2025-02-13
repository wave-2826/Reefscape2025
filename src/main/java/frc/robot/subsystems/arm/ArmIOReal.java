package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.util.SparkUtil.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.util.SparkUtil;

public class ArmIOReal implements ArmIO {
    protected SparkFlex elevatorHeightMotor1;
    protected SparkFlex elevatorHeightMotor2;
    protected SparkMax armPitchMotor;
    protected SparkMax armWristMotor;
    protected SparkFlex endEffectorMotor;

    private SparkClosedLoopController elevatorHeightController1;
    private SparkClosedLoopController elevatorHeightController2;
    private SparkClosedLoopController armPitchController;
    private SparkClosedLoopController armWristController;
    private SparkClosedLoopController endEffectorController;

    private RelativeEncoder elevatorHeightEncoder1;
    private RelativeEncoder elevatorHeightEncoder2;
    private AbsoluteEncoder armPitchEncoder;
    private AbsoluteEncoder armWristEncoder;
    private RelativeEncoder endEffectorEncoder;

    private SparkLimitSwitch elevatorResetSwitch;
    private SparkLimitSwitch verticalGamePieceSwitch;
    private SparkLimitSwitch horizontalGamePieceSwitch;

    private Debouncer elevatorConnectedDebouncer = new Debouncer(0.25);
    private Debouncer armPitchConnectedDebouncer = new Debouncer(0.25);
    private Debouncer armWristConnectedDebouncer = new Debouncer(0.25);
    private Debouncer endEffectorConnectedDebouncer = new Debouncer(0.25);

    public ArmIOReal() {
        // Create motor contorllers 
        elevatorHeightMotor1 = new SparkFlex(ArmConstants.ElevatorConstants.elevatorHeightMotor1Id,
            MotorType.kBrushless);
        elevatorHeightMotor2 = new SparkFlex(ArmConstants.ElevatorConstants.elevatorHeightMotor2Id,
            MotorType.kBrushless);

        armPitchMotor = new SparkMax(ArmConstants.ShoulderConstants.armPitchMotorId, MotorType.kBrushless);
        armWristMotor = new SparkMax(ArmConstants.ShoulderConstants.armWristMotorId, MotorType.kBrushless);

        endEffectorMotor = new SparkFlex(ArmConstants.EndEffectorConstants.endEffectorMotorId, MotorType.kBrushless);

        // Configure motor controllers
        SparkFlexConfig elevatorMotorConfig = new SparkFlexConfig();
        elevatorMotorConfig.closedLoop.apply(ArmConstants.ElevatorConstants.elevatorPID.getConfig())
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        elevatorMotorConfig.encoder
            .positionConversionFactor(ArmConstants.ElevatorConstants.elevatorPositionConversionFactor)
            .velocityConversionFactor(ArmConstants.ElevatorConstants.elevatorVelocityConversionFactor);
        // We use the forward limit switch to reset the encoder, but don't actually disable actuation when it's pressed.
        elevatorMotorConfig.limitSwitch.forwardLimitSwitchEnabled(false);
        elevatorMotorConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ArmConstants.ElevatorConstants.elevatorMotorCurrentLimit)
            .voltageCompensation(Constants.voltageCompensation)
            .inverted(ArmConstants.ElevatorConstants.elevatorMotorInverted);
        elevatorMotorConfig.signals.apply(SparkUtil.defaultSignals).limitsPeriodMs(20)
            .primaryEncoderPositionAlwaysOn(true).primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(20).primaryEncoderVelocityPeriodMs(20);

        tryUntilOk(elevatorHeightMotor1, 5, () -> elevatorHeightMotor1.configure(elevatorMotorConfig,
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(elevatorHeightMotor2, 5, () -> elevatorHeightMotor2.configure(elevatorMotorConfig,
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        SparkMaxConfig armPitchConfig = new SparkMaxConfig();
        armPitchConfig.closedLoop.apply(ArmConstants.ShoulderConstants.armPitchPID.getConfig())
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        armPitchConfig.absoluteEncoder
            .positionConversionFactor(ArmConstants.ShoulderConstants.pitchPositionConversionFactor)
            .velocityConversionFactor(ArmConstants.ShoulderConstants.pitchVelocityConversionFactor);
        armPitchConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ArmConstants.ShoulderConstants.pitchMotorCurrentLimit)
            .voltageCompensation(Constants.voltageCompensation)
            .inverted(ArmConstants.ShoulderConstants.pitchMotorInverted);
        armPitchConfig.signals.apply(SparkUtil.defaultSignals) //
            .absoluteEncoderPositionAlwaysOn(true).absoluteEncoderVelocityAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs(20).absoluteEncoderVelocityPeriodMs(20);

        tryUntilOk(armPitchMotor, 5, () -> armPitchMotor.configure(armPitchConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));

        SparkMaxConfig armWristConfig = new SparkMaxConfig();
        armWristConfig.closedLoop.apply(ArmConstants.ShoulderConstants.armWristPID.getConfig())
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        armWristConfig.absoluteEncoder
            .positionConversionFactor(ArmConstants.ShoulderConstants.wristPositionConversionFactor)
            .velocityConversionFactor(ArmConstants.ShoulderConstants.wristVelocityConversionFactor);
        armWristConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ArmConstants.ShoulderConstants.wristMotorCurrentLimit)
            .voltageCompensation(Constants.voltageCompensation)
            .inverted(ArmConstants.ShoulderConstants.wristMotorInverted);
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
            .inverted(ArmConstants.EndEffectorConstants.endEffectorMotorInverted);
        endEffectorConfig.signals.apply(SparkUtil.defaultSignals) //
            .primaryEncoderPositionAlwaysOn(false).primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20);

        tryUntilOk(endEffectorMotor, 5, () -> endEffectorMotor.configure(endEffectorConfig,
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        // Register the tunable PIDs
        ArmConstants.ElevatorConstants.elevatorPID.configureSparkOnChange(elevatorHeightMotor1);
        ArmConstants.ElevatorConstants.elevatorPID.configureSparkOnChange(elevatorHeightMotor2);
        ArmConstants.ShoulderConstants.armPitchPID.configureSparkOnChange(armPitchMotor);
        ArmConstants.ShoulderConstants.armWristPID.configureSparkOnChange(armWristMotor);
        ArmConstants.EndEffectorConstants.endEffectorPID.configureSparkOnChange(endEffectorMotor);

        // Create closed loop controllers
        elevatorHeightController1 = elevatorHeightMotor1.getClosedLoopController();
        elevatorHeightController2 = elevatorHeightMotor2.getClosedLoopController();
        armPitchController = armPitchMotor.getClosedLoopController();
        armWristController = armWristMotor.getClosedLoopController();
        endEffectorController = endEffectorMotor.getClosedLoopController();

        // Create sensors
        elevatorResetSwitch = elevatorHeightMotor1.getForwardLimitSwitch();

        elevatorHeightEncoder1 = elevatorHeightMotor1.getEncoder();
        elevatorHeightEncoder2 = elevatorHeightMotor2.getEncoder();

        armPitchEncoder = armPitchMotor.getAbsoluteEncoder();
        armWristEncoder = armWristMotor.getAbsoluteEncoder();

        endEffectorEncoder = endEffectorMotor.getEncoder();

        verticalGamePieceSwitch = armWristMotor.getForwardLimitSwitch();
        horizontalGamePieceSwitch = armWristMotor.getReverseLimitSwitch();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        // Update elevator height motor inputs
        sparkStickyFault = false;

        // TODO: Slightly more complicated logic to improve the precision of resetting by using our
        // direction and resetting on the falling edge to the sensor top/bottom
        ifOk(elevatorHeightMotor1, elevatorResetSwitch::isPressed, (v) -> inputs.elevatorLimitSwitchActive = v);
        if(inputs.elevatorLimitSwitchActive) {
            tryUntilOk(elevatorHeightMotor1, 1,
                () -> elevatorHeightEncoder1.setPosition(ArmConstants.ElevatorConstants.resetSwitchHeight.in(Meters)));
            tryUntilOk(elevatorHeightMotor2, 1,
                () -> elevatorHeightEncoder2.setPosition(ArmConstants.ElevatorConstants.resetSwitchHeight.in(Meters)));
        }

        // We average the two encoders' readings because they may not be perfectly in sync.
        ifOk(new SparkBase[] {
            elevatorHeightMotor1, elevatorHeightMotor2
        }, new DoubleSupplier[] {
            elevatorHeightEncoder1::getPosition, elevatorHeightEncoder2::getPosition
        }, (v) -> inputs.elevatorHeightMeters = (v[0] + v[1]) / 2.);

        inputs.elevatorMotorsConnected = elevatorConnectedDebouncer.calculate(!sparkStickyFault);

        // Update arm pitch motor inputs
        sparkStickyFault = false;
        ifOk(armPitchMotor, armPitchEncoder::getPosition, (v) -> inputs.armPitchPosition = Rotation2d.fromRadians(v));
        ifOk(armPitchMotor, armPitchEncoder::getVelocity, (v) -> inputs.armPitchVelocity = v);
        inputs.armPitchMotorConnected = armPitchConnectedDebouncer.calculate(!sparkStickyFault);

        // Update arm wrist motor inputs
        sparkStickyFault = false;
        ifOk(armWristMotor, armWristEncoder::getPosition, (v) -> inputs.armWristPosition = Rotation2d.fromRadians(v));
        ifOk(armWristMotor, armWristEncoder::getVelocity, (v) -> inputs.armWristVelocity = v);
        if(Math.abs(inputs.armWristPosition.getDegrees() % 180) < 45.) {
            inputs.gamePiecePresent = verticalGamePieceSwitch.isPressed();
        } else {
            inputs.gamePiecePresent = horizontalGamePieceSwitch.isPressed();
        }
        inputs.armWristMotorConnected = armWristConnectedDebouncer.calculate(!sparkStickyFault);

        // Update end effector motor inputs
        sparkStickyFault = false;
        ifOk(endEffectorMotor, endEffectorEncoder::getVelocity, (v) -> inputs.endEffectorVelocity = v);
        inputs.endEffectorMotorConnected = endEffectorConnectedDebouncer.calculate(!sparkStickyFault);
    }

    @Override
    public void setElevatorHeight(double heightMeters) {
        elevatorHeightController1.setReference(heightMeters, ControlType.kPosition);
        elevatorHeightController2.setReference(heightMeters, ControlType.kPosition);
    }

    @Override
    public void setArmPitchPosition(Rotation2d position) {
        armPitchController.setReference(position.getRadians(), ControlType.kPosition);
    }

    @Override
    public void setArmWristPosition(Rotation2d position) {
        armWristController.setReference(position.getRadians(), ControlType.kPosition,
            ArmConstants.ShoulderConstants.armWristPositionSlot);
    }

    @Override
    public void setArmWristVelocity(double velocityRadPerSecond) {
        armWristController.setReference(velocityRadPerSecond, ControlType.kVelocity,
            ArmConstants.ShoulderConstants.armWristVelocitySlot);
    }

    @Override
    public void setEndEffectorVelocity(double velocityRadPerSecond) {
        endEffectorController.setReference(velocityRadPerSecond, ControlType.kVelocity);
    }
}
