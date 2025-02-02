package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.util.SparkUtil;

public class ArmIOReal implements ArmIO {
    SparkFlex elevatorHeightMotor1;
    SparkFlex elevatorHeightMotor2;
    SparkMax armPitchMotor;
    SparkMax armWristMotor;
    SparkFlex endEffectorMotor;

    RelativeEncoder elevatorHeightEncoder1;
    RelativeEncoder elevatorHeightEncoder2;
    AbsoluteEncoder armPitchEncoder;
    AbsoluteEncoder armWristEncoder;
    RelativeEncoder endEffectorEncoder;

    SparkLimitSwitch elevatorResetSwitch;
    SparkLimitSwitch verticalGamePieceSwitch;
    SparkLimitSwitch horizontalGamePieceSwitch;

    public ArmIOReal() {
        // Create motor contorllers 
        elevatorHeightMotor1 = new SparkFlex(ArmConstants.ElevatorConstants.elevatorHeightMotor1Id,
            MotorType.kBrushless);
        elevatorHeightMotor2 = new SparkFlex(ArmConstants.ElevatorConstants.elevatorHeightMotor2Id,
            MotorType.kBrushless);

        armPitchMotor = new SparkMax(ArmConstants.PitchWristConstants.armPitchMotorId, MotorType.kBrushless);
        armWristMotor = new SparkMax(ArmConstants.PitchWristConstants.armWristMotorId, MotorType.kBrushless);

        endEffectorMotor = new SparkFlex(ArmConstants.EndEffectorConstants.endEffectorMotorId, MotorType.kBrushless);

        // Configure motor controllers
        SparkFlexConfig elevatorMotorConfig = new SparkFlexConfig();
        elevatorMotorConfig.closedLoop.apply(ArmConstants.ElevatorConstants.elevatorHeightClosedLoopConfig)
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
        armPitchConfig.closedLoop.apply(ArmConstants.PitchWristConstants.armPitchClosedLoopConfig)
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        armPitchConfig.absoluteEncoder
            .positionConversionFactor(ArmConstants.PitchWristConstants.pitchPositionConversionFactor)
            .velocityConversionFactor(ArmConstants.PitchWristConstants.pitchVelocityConversionFactor);
        armPitchConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ArmConstants.PitchWristConstants.pitchMotorCurrentLimit)
            .voltageCompensation(Constants.voltageCompensation)
            .inverted(ArmConstants.PitchWristConstants.pitchMotorInverted);
        armPitchConfig.signals.apply(SparkUtil.defaultSignals) //
            .absoluteEncoderPositionAlwaysOn(true).absoluteEncoderVelocityAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs(20).absoluteEncoderVelocityPeriodMs(20);

        tryUntilOk(armPitchMotor, 5, () -> armPitchMotor.configure(armPitchConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));

        SparkMaxConfig armWristConfig = new SparkMaxConfig();
        armWristConfig.closedLoop.apply(ArmConstants.PitchWristConstants.armWristClosedLoopConfig)
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        armWristConfig.absoluteEncoder
            .positionConversionFactor(ArmConstants.PitchWristConstants.wristPositionConversionFactor)
            .velocityConversionFactor(ArmConstants.PitchWristConstants.wristVelocityConversionFactor);
        armWristConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ArmConstants.PitchWristConstants.wristMotorCurrentLimit)
            .voltageCompensation(Constants.voltageCompensation)
            .inverted(ArmConstants.PitchWristConstants.wristMotorInverted);
        // We use the forward and backward limit switch for the vertical and horizontal game piece
        // detectors respectvely. We don't want them to act as actual limit switches.
        armWristConfig.limitSwitch.forwardLimitSwitchEnabled(false).reverseLimitSwitchEnabled(false);
        armWristConfig.signals.apply(SparkUtil.defaultSignals) //
            .limitsPeriodMs(20).absoluteEncoderPositionAlwaysOn(true).absoluteEncoderVelocityAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs(20).absoluteEncoderVelocityPeriodMs(20);

        tryUntilOk(armWristMotor, 5, () -> armWristMotor.configure(armWristConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));

        SparkMaxConfig endEffectorConfig = new SparkMaxConfig();
        endEffectorConfig.closedLoop.apply(ArmConstants.EndEffectorConstants.endEffectorClosedLoopConfig)
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
    public void updateElevatorPosition() {
        // TODO: Slightly more complicated logic to improve the precision of resetting by using our
        // direction and resetting on the falling edge to the sensor top/bottom
        if(elevatorResetSwitch.isPressed()) {
            elevatorHeightEncoder1.setPosition(ArmConstants.ElevatorConstants.resetSwitchHeight.in(Meters));
            elevatorHeightEncoder2.setPosition(ArmConstants.ElevatorConstants.resetSwitchHeight.in(Meters));
        }
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.elevatorLimitSwitchActive = elevatorResetSwitch.isPressed();

        inputs.armPitchPosition = Rotation2d.fromRadians(armPitchEncoder.getPosition());
        inputs.armPitchVelocity = armPitchEncoder.getVelocity();

        inputs.armWristPosition = Rotation2d.fromRadians(armWristEncoder.getPosition());
        inputs.armWristVelocity = armWristEncoder.getVelocity();

        inputs.endEffectorVelocity = endEffectorEncoder.getVelocity();

        inputs.gamePiecePresent = false; // TODO: Detect game piece based on wrist angle
    }
}
