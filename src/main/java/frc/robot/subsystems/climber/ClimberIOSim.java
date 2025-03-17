package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;

/** I don't really care about accurately simulating the climber, so we fake it. */
public class ClimberIOSim implements ClimberIO {
    private Rotation2d rotation;

    public ClimberIOSim() {
        rotation = Rotation2d.kZero;
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberMotorConnected = true;
        inputs.climberAbsolutePosition = rotation;
    }

    @Override
    public void runClimberOpenLoop(double power) {
        rotation = rotation.plus(Rotation2d.fromDegrees(power * 40 * 0.02));
    }

    @Override
    public void setClimberTargetAngle(Rotation2d angle) {
        rotation = angle;
    }
}
