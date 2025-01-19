package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
    /**
     * Defines the runtime mode used by AdvantageKit. The mode is always "real" when running on a roboRIO. Change the
     * value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a file).
     */
    public static final Mode simMode = Mode.SIM;

    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,
        /** Running a physics simulator. */
        SIM,
        /** Replaying from a log file. */
        REPLAY
    };

    public static final double voltageCompensation = 12.0;
}
