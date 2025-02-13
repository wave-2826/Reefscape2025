package frc.robot.subsystems.arm;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

public record ArmState(Rotation2d pitch, Distance height, WristRotation wristRotation,
    EndEffectorState endEffectorState) {
    public enum WristRotation {
        Vertical(Rotation2d.fromDegrees(0)), Horizontal(Rotation2d.fromDegrees(90));
        // TODO: Showing off mode, as per B-G's recommendation

        Rotation2d rotation;

        WristRotation(Rotation2d rotation) {
            this.rotation = rotation;
        }
    }

    /**
     * Maybe there's a better way to represent this state? It's tricky because Java enums don't support values.
     */
    public class EndEffectorState {
        private enum Mode {
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
}
