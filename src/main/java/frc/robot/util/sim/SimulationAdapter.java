package frc.robot.util.sim;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public interface SimulationAdapter {
    /** Run before any robot code starts. */
    public default void preInit() {
    }

    /** Run after all robot code has been initialized. */
    public default void postInit() {
    }

    /** Run every tick in simulation */
    public default void tick() {
    }

    /** Adjusts the autonomous command. */
    public default Command transformAutoCommand(Command command, RobotContainer robotContainer) {
        return command;
    }
}
