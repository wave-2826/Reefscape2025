package frc.robot.subsystems.arm;

import java.util.OptionalDouble;

/**
 * Maybe there's a better way to represent this state? It's tricky because Java enums don't support values.
 */
public class EndEffectorState {
    public enum Mode {
        VelocityControl, Hold
    }

    private Mode mode;
    private double velocityRadPerSecond;

    public OptionalDouble getVelocityControl() {
        if(mode == Mode.VelocityControl) { return OptionalDouble.of(velocityRadPerSecond); }
        return OptionalDouble.empty();
    }

    public boolean isHold() {
        return mode == Mode.Hold;
    }

    private EndEffectorState(Mode mode, double velocityRadPerSecond) {
        this.mode = mode;
        this.velocityRadPerSecond = velocityRadPerSecond;
    }

    public static EndEffectorState hold() {
        return new EndEffectorState(Mode.Hold, 0);
    }

    /**
     * @param velocityRadPerSecond The end effector velocity. Positive velocity is outward.
     * @return
     */
    public static EndEffectorState velocity(double velocityRadPerSecond) {
        return new EndEffectorState(Mode.VelocityControl, velocityRadPerSecond);
    }
}
