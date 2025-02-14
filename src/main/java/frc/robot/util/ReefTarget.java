package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.FieldConstants.ReefBranch;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.EndEffectorState;

public record ReefTarget(ReefBranch branch, ReefLevel level) {
    public ArmState getArmState() {
        return new ArmState(new Rotation2d(level.pitch), level.height, ArmState.WristRotation.Vertical,
            new EndEffectorState(EndEffectorState.Mode.Hold));
    }

    @Override
    public boolean equals(Object obj) {
        if(this == obj) { return true; }
        if(obj == null || getClass() != obj.getClass()) { return false; }
        ReefTarget other = (ReefTarget) obj;
        return branch == other.branch && level == other.level;
    }
}
