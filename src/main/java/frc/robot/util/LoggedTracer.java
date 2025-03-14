package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

/**
 * Utility class for logging code execution times. From 6328's implementation:
 * https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/main/src/main/java/org/littletonrobotics/frc2025/util/LoggedTracer.java
 */
public class LoggedTracer {
    private LoggedTracer() {
    }

    private static double startTime = -1.0;

    /** Reset the clock. */
    public static void reset() {
        startTime = Timer.getFPGATimestamp();
    }

    /** Save the time elapsed since the last reset or record. */
    public static void record(String epochName) {
        double now = Timer.getFPGATimestamp();
        Logger.recordOutput("LoggedTracer/" + epochName + "MS", (now - startTime) * 1000.0);
        startTime = now;
    }
}
