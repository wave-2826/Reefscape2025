package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class SuppliedWait extends Command {
    protected Timer m_timer = new Timer();
    private final DoubleSupplier m_duration;

    public SuppliedWait(DoubleSupplier seconds) {
        m_duration = seconds;
    }

    @Override
    public void initialize() {
        m_timer.restart();
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_duration.getAsDouble());
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
