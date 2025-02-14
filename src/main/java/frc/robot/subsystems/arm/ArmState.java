package frc.robot.subsystems.arm;

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
}
