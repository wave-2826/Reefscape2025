package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

public record ArmState(Rotation2d pitch, Distance height, WristRotation wristRotation,
    EndEffectorState endEffectorState) {
    /** The rotation of the wrist. The coral is 90 degrees offset from this. */
    public enum WristRotation {
        /**
         * Horizontal wrist rotation / vertical coal rotation. This is the state used for picking up from the transport.
         */
        Horizontal(Rotation2d.fromDegrees(0)),
        /** Vertical wrist rotation / horizontal coral rotation. */
        Vertical(Rotation2d.fromDegrees(90)),

        /**
         * Horizontal wrist rotation / vertical coal rotation, 180 degrees offset. This is the state used for scoring
         * L2-L4.
         */
        HorizontalFlipped(Rotation2d.fromDegrees(180)),
        /** Vertical wrist rotation / horizontal coral rotation, 180 degrees offset. */
        VerticalFlipped(Rotation2d.fromDegrees(270));

        // TODO: Showing off mode, as per B-G's recommendation

        Rotation2d rotation;

        WristRotation(Rotation2d rotation) {
            this.rotation = rotation;
        }

        public WristRotation previous() {
            return values()[(ordinal() - 1 + values().length) % values().length];
        }

        public WristRotation next() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    public ArmState withHeight(Distance newHeight) {
        return new ArmState(pitch, newHeight, wristRotation, endEffectorState);
    }

    public ArmState withPitch(Rotation2d newPitch) {
        return new ArmState(newPitch, height, wristRotation, endEffectorState);
    }

    public ArmState withEndEffector(EndEffectorState newEndEffectorState) {
        return new ArmState(pitch, height, wristRotation, newEndEffectorState);
    }

    public ArmState withWrist(WristRotation newRotation) {
        return new ArmState(pitch, height, newRotation, endEffectorState);
    }
}
