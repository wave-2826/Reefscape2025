package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

public record ArmState(Rotation2d pitch, Distance height, WristRotation wristRotation,
    EndEffectorState endEffectorState) {
    /** The rotation of the wrist. The coral is 90 degrees offset from this. */
    public enum WristRotation {
        /** Vertical wrist rotation / horizontal coral rotation. */
        Vertical(Rotation2d.fromDegrees(90)),
        /**
         * Horizontal wrist rotation / vertical coal rotation. This is the state used for picking up from the transport.
         */
        Horizontal(Rotation2d.fromDegrees(0)),

        /** Vertical wrist rotation / horizontal coral rotation, 180 degrees offset. */
        VerticalFlipped(Rotation2d.fromDegrees(270)),
        /**
         * Horizontal wrist rotation / vertical coal rotation, 180 degrees offset. This is the state used for scoring
         * L2-L4.
         */
        HorizontalFlipped(Rotation2d.fromDegrees(180));

        // TODO: Showing off mode, as per B-G's recommendation

        Rotation2d rotation;

        WristRotation(Rotation2d rotation) {
            this.rotation = rotation;
        }
    }

    public ArmState withPitch(Rotation2d newPitch) {
        return new ArmState(newPitch, height, wristRotation, endEffectorState);
    }
}
