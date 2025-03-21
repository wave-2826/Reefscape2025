package frc.robot.subsystems.climber;

import static frc.robot.util.SparkUtil.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.util.SparkUtil;

public class ClimberIOReal implements ClimberIO {
    protected final SparkMax climberMotor;
    protected final RelativeEncoder climberEncoder;
    protected final AbsoluteEncoder climberAbsoluteEncoder;
    protected final SparkClosedLoopController climberMotorController;

    private Debouncer climberConnectedDebouncer = new Debouncer(0.25);

    public ClimberIOReal() {
        climberMotor = new SparkMax(ClimberConstants.climberMotorId, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();

        config.closedLoop.apply(ClimberConstants.climberPID.getConfig())
            // We don't use closed-loop control directly with the absolute encoder because
            // otherwise we can get stuck turning the motor into an invalid state if the climber
            // arm is resting on an object (e.g. the cage...).
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.absoluteEncoder.positionConversionFactor(ClimberConstants.climberAbsolutePositionFactor)
            .velocityConversionFactor(ClimberConstants.climberAbsoluteVelocityFactor)
            .zeroOffset(Constants.isSim ? 0 : ClimberConstants.climberZeroAngle).zeroCentered(true);
        config.encoder.positionConversionFactor(ClimberConstants.climberPositionConversionFactor)
            .velocityConversionFactor(ClimberConstants.climberVelocityConversionFactor);
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(ClimberConstants.climberMotorCurrentLimit)
            .voltageCompensation(Constants.voltageCompensation).inverted(ClimberConstants.climberMotorInverted);
        config.signals.apply(SparkUtil.defaultSignals) //
            .primaryEncoderPositionAlwaysOn(false).primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20).absoluteEncoderPositionAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs(20);

        tryUntilOk(climberMotor, 5,
            () -> climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        ClimberConstants.climberPID.configureSparkOnChange(climberMotor);

        climberMotorController = climberMotor.getClosedLoopController();

        climberEncoder = climberMotor.getEncoder();
        climberAbsoluteEncoder = climberMotor.getAbsoluteEncoder();

        resetToAbsolute();
    }

    @Override
    public void resetToAbsolute() {
        tryUntilOk(climberMotor, 5, () -> climberEncoder.setPosition(climberAbsoluteEncoder.getPosition()));
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        sparkStickyFault = false;
        ifOk(climberMotor, climberAbsoluteEncoder::getPosition,
            a -> inputs.climberAbsolutePosition = Rotation2d.fromRadians(a));
        inputs.climberMotorConnected = climberConnectedDebouncer.calculate(!sparkStickyFault);
    }

    @Override
    public void setClimberTargetAngle(Rotation2d angle) {
        // Lmaoo wtf
        double theta = Math.PI * (3 / 4.) - angle.getRadians();
        double requiredClimberStrapLength = Math
            .sqrt(ClimberConstants.climberArmStrapPosition * ClimberConstants.climberArmStrapPosition
                + ClimberConstants.climberPulleyToPivotDistance * ClimberConstants.climberPulleyToPivotDistance
                - 2 * ClimberConstants.climberArmStrapPosition * ClimberConstants.climberPulleyToPivotDistance
                    * Math.cos(theta));
        double climberCircumference = 2 * Math.PI * ClimberConstants.climberPulleyRadius;
        Logger.recordOutput("Climber/RequiredStrapLength", requiredClimberStrapLength);
        double requiredPulleyPosition = (ClimberConstants.climberRestingLength - requiredClimberStrapLength)
            / climberCircumference * 2 * Math.PI;
        Logger.recordOutput("Climber/PulleyPositionTarget", requiredPulleyPosition);
        climberMotorController.setReference(requiredPulleyPosition, ControlType.kPosition);
    }

    @Override
    public void runClimberOpenLoop(double power) {
        climberMotorController.setReference(power, ControlType.kDutyCycle);
    }

    @Override
    public void setClimberBrakeMode(boolean enable) {
        SparkMaxConfig newConfig = new SparkMaxConfig();
        newConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);

        tryUntilOk(climberMotor, 5, () -> climberMotor.configure(newConfig, ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters));
    }
}
