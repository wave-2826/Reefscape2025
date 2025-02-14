package frc.robot.subsystems.arm;

import java.util.Optional;

/**
 * Maybe there's a better way to represent this state? It's tricky because Java enums don't support values.
 */
public class EndEffectorState {
    public enum Mode {
        VelocityControl, Hold
    }

    private Mode mode;
    private double velocityRadPerSecond;

    public Optional<Double> getVelocityControl() {
        if(mode == Mode.VelocityControl) { return Optional.of(velocityRadPerSecond); }
        return Optional.empty();
    }

    public EndEffectorState(Mode mode, double velocityRadPerSecond) {
        this.mode = mode;
        this.velocityRadPerSecond = velocityRadPerSecond;
    }

    public EndEffectorState(Mode mode) {
        this(mode, 0.0);
    }
}