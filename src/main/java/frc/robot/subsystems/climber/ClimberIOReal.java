package frc.robot.subsystems.climber;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.util.SparkUtil;

public class ClimberIOReal implements ClimberIO {
    SparkMax climberMotor;
    AbsoluteEncoder climberEncoder;

    SparkClosedLoopController climberMotorController;

    public ClimberIOReal() {
        climberMotor = new SparkMax(ClimberConstants.climberMotorId, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();

        config.closedLoop.apply(ClimberConstants.climberPID.getConfig()).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
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

        climberEncoder = climberMotor.getAbsoluteEncoder();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        sparkStickyFault = false;
        ifOk(climberMotor, climberEncoder::getPosition, a -> inputs.climberPosition = Rotation2d.fromRadians(a));
        inputs.climberMotorConnected = !sparkStickyFault;
    }

    @Override
    public void setClimberTargetAngle(Rotation2d angle) {
        climberMotorController.setReference(angle.getRadians(), ControlType.kPosition);
    }

    @Override
    public void setClimberBrakeMode(boolean enable) {
        SparkMaxConfig newConfig = new SparkMaxConfig();
        newConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);

        tryUntilOk(climberMotor, 5, () -> climberMotor.configure(newConfig, ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters));
    }
}
