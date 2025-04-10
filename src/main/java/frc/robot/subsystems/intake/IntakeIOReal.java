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

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.util.SparkUtil;

public class IntakeIOReal implements IntakeIO {
    protected final SparkMax pitchMotor;
    protected final SparkMax transportMotor;
    protected final SparkFlex powerMotor;

    protected final DigitalInput intakeSensor;
    protected final DigitalInput middleSensor;
    protected final DigitalInput endSensor;

    protected final SparkClosedLoopController pitchController;
    protected final SparkClosedLoopController powerController;
    protected final SparkClosedLoopController transportController;

    protected final AbsoluteEncoder absolutePitchSensor;
    protected final RelativeEncoder powerEncoder;
    protected final RelativeEncoder transportEncoder;

    private final Debouncer pitchConnectedDebouncer = new Debouncer(0.25);
    private final Debouncer powerConnectedDebouncer = new Debouncer(0.25);
    private final Debouncer transportConnectedDebouncer = new Debouncer(0.25);

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

        // Register motor faults
        registerSparkFaultAlerts(pitchMotor, "Intake pitch motor");
        registerSparkFaultAlerts(powerMotor, "Intake power motor");
        registerSparkFaultAlerts(transportMotor, "Transport motor");

        pitchController = pitchMotor.getClosedLoopController();
        powerController = powerMotor.getClosedLoopController();
        transportController = transportMotor.getClosedLoopController();

        absolutePitchSensor = pitchMotor.getAbsoluteEncoder();
        powerEncoder = powerMotor.getEncoder();
        transportEncoder = transportMotor.getEncoder();

        intakeSensor = new DigitalInput(IntakeConstants.intakeSensorDIOPort);
        middleSensor = new DigitalInput(IntakeConstants.middleSensorDIOPort);
        endSensor = new DigitalInput(IntakeConstants.endSensorDIOPort);
    }

    @Override
    public void runVelocity(double intakePower, double transportPower) {
        powerController.setReference(Units.rotationsPerMinuteToRadiansPerSecond(intakePower * 2900),
            ControlType.kVelocity);
        transportController.setReference(Units.rotationsPerMinuteToRadiansPerSecond(transportPower * 9500),
            ControlType.kVelocity);
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
        pitchController.setReference(-power, ControlType.kDutyCycle);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeSensorTriggered = !intakeSensor.get();
        inputs.middleSensorTriggered = !middleSensor.get();
        inputs.endSensorTriggered = !endSensor.get();

        sparkStickyFault = false;
        inputs.intakePitch = Rotation2d.fromRadians(absolutePitchSensor.getPosition());
        inputs.intakePitchMotorConnected = pitchConnectedDebouncer.calculate(!sparkStickyFault);

        sparkStickyFault = false;
        inputs.intakeWheelSpeed = powerEncoder.getVelocity();
        inputs.intakePowerMotorConnected = powerConnectedDebouncer.calculate(!sparkStickyFault);

        sparkStickyFault = false;
        inputs.transportWheelSpeed = transportEncoder.getVelocity();
        inputs.transportMotorConnected = transportConnectedDebouncer.calculate(!sparkStickyFault);
    }
}
