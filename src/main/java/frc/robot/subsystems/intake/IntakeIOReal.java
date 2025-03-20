package frc.robot.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.util.SparkUtil;

public class IntakeIOReal implements IntakeIO {
    protected final SparkMax pitchMotor;
    protected final SparkMax transportMotor;
    protected final SparkFlex powerMotor;

    protected final DigitalInput transportSensor;
    protected final DigitalInput intakeSensor;

    protected final SparkClosedLoopController pitchController;
    protected final SparkClosedLoopController powerController;
    protected final SparkClosedLoopController transportController;

    protected final AbsoluteEncoder absolutePitchSensor;
    protected final RelativeEncoder powerEncoder;

    public IntakeIOReal() {
        pitchMotor = new SparkMax(IntakeConstants.intakePitchMotorId, MotorType.kBrushless);
        powerMotor = new SparkFlex(IntakeConstants.intakeDriveMotorId, MotorType.kBrushless);
        transportMotor = new SparkMax(IntakeConstants.transportDriveMotorId, MotorType.kBrushless);

        SparkMaxConfig pitchConfig = new SparkMaxConfig();
        pitchConfig.closedLoop.apply(IntakeConstants.pitchPID.getConfig())
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        pitchConfig.absoluteEncoder.positionConversionFactor(IntakeConstants.pitchAbsolutePositionFactor)
            .velocityConversionFactor(IntakeConstants.pitchAbsoluteVelocityFactor)
            .zeroOffset(Constants.isSim ? 0 : IntakeConstants.intakeZeroAngle).zeroCentered(true)
            .inverted(IntakeConstants.pitchEncoderInverted);
        pitchConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(IntakeConstants.pitchMotorCurrentLimit)
            .voltageCompensation(Constants.voltageCompensation).inverted(IntakeConstants.pitchMotorInverted);
        pitchConfig.signals.apply(SparkUtil.defaultSignals) //
            .primaryEncoderPositionAlwaysOn(false).primaryEncoderVelocityAlwaysOn(false)
            .absoluteEncoderPositionAlwaysOn(true).absoluteEncoderPositionPeriodMs(20);
        pitchConfig.closedLoopRampRate(0.2);

        tryUntilOk(pitchMotor, 5,
            () -> pitchMotor.configure(pitchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        SparkMaxConfig powerConfig = new SparkMaxConfig();
        powerConfig.closedLoop.apply(IntakeConstants.powerPID.getConfig());
        powerConfig.encoder.positionConversionFactor(IntakeConstants.powerPositionConversionFactor)
            .velocityConversionFactor(IntakeConstants.powerVelocityConversionFactor);
        powerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(IntakeConstants.powerMotorCurrentLimit)
            .voltageCompensation(Constants.voltageCompensation).inverted(IntakeConstants.powerMotorInverted);
        powerConfig.signals.apply(SparkUtil.defaultSignals) //
            .primaryEncoderPositionAlwaysOn(true).primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(100).primaryEncoderVelocityPeriodMs(100);
        powerConfig.closedLoopRampRate(0.3);

        tryUntilOk(powerMotor, 5,
            () -> powerMotor.configure(powerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        SparkMaxConfig transportConfig = new SparkMaxConfig();
        transportConfig.closedLoop.apply(IntakeConstants.transportPID.getConfig());
        transportConfig.encoder.positionConversionFactor(IntakeConstants.transportPositionConversionFactor)
            .velocityConversionFactor(IntakeConstants.transportVelocityConversionFactor);
        transportConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(IntakeConstants.transportMotorCurrentLimit)
            .voltageCompensation(Constants.voltageCompensation).inverted(IntakeConstants.transportMotorInverted);
        transportConfig.signals.apply(SparkUtil.defaultSignals) //
            .primaryEncoderPositionAlwaysOn(true).primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(100).primaryEncoderVelocityPeriodMs(100);
        transportConfig.closedLoopRampRate(0.3);

        tryUntilOk(transportMotor, 5, () -> transportMotor.configure(transportConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));

        IntakeConstants.pitchPID.configureSparkOnChange(pitchMotor);
        IntakeConstants.powerPID.configureSparkOnChange(powerMotor);
        IntakeConstants.transportPID.configureSparkOnChange(transportMotor);

        pitchController = pitchMotor.getClosedLoopController();
        powerController = powerMotor.getClosedLoopController();
        transportController = transportMotor.getClosedLoopController();

        absolutePitchSensor = pitchMotor.getAbsoluteEncoder();
        powerEncoder = powerMotor.getEncoder();

        intakeSensor = new DigitalInput(IntakeConstants.intakeSensorDIOPort);
        transportSensor = new DigitalInput(IntakeConstants.transportSensorDIOPort);
    }

    @Override
    public void runVelocity(double intakePower, double transportPower) {
        powerController.setReference(intakePower * 3500., ControlType.kVelocity);
        transportController.setReference(transportPower * 5000., ControlType.kVelocity);
    }

    @Override
    public void setIntakePitch(Rotation2d pitch) {
        pitchController.setReference(pitch.getRadians(), ControlType.kPosition);
    }

    @Override
    public void setIntakeCoast() {
        pitchMotor.stopMotor();
    }

    @Override
    public void overridePitchPower(double power) {
        pitchMotor.set(-power);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeSensorTriggered = !intakeSensor.get();
        inputs.transortSensorTriggered = !transportSensor.get();
        inputs.intakePitch = Rotation2d.fromRadians(absolutePitchSensor.getPosition());
        inputs.intakeWheelSpeed = powerEncoder.getVelocity();
    }
}
