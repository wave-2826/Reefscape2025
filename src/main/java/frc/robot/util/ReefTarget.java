package frc.robot.util;

import frc.robot.FieldConstants.ReefBranch;
import frc.robot.FieldConstants.ReefLevel;

public record ReefTarget(ReefBranch branch, ReefLevel level) {
    @Override
    public boolean equals(Object obj) {
        if(this == obj) { return true; }
        if(obj == null || getClass() != obj.getClass()) { return false; }
        ReefTarget other = (ReefTarget) obj;
        return branch == other.branch && level == other.level;
    }

    @Override
    public final String toString() {
        return level.toString() + branch.toString();
    }
}
