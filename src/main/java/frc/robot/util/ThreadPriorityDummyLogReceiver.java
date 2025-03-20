package frc.robot.util;

import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;

import edu.wpi.first.wpilibj.Threads;

/**
 * A dummy log reciever used to adjust the thread priority of the log recieving thread in AdvantageKit. Used if
 * real-time thread priority is enabled.
 */
public class ThreadPriorityDummyLogReceiver implements LogDataReceiver {
    @Override
    public void start() {
        Threads.setCurrentThreadPriority(true, 1);
    }

    @Override
    public void putTable(LogTable table) throws InterruptedException {
    }
}
