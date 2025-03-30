package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

public class LoggedCommand extends Command {
    private String name;
    private Command child;
    private static Map<String, Integer> commandCounts = new HashMap<>();

    public LoggedCommand(String name, Command child) {
        this.name = name;
        this.child = child;
    }

    private void log(boolean running) {
        int count = commandCounts.getOrDefault(name, 0) + (running ? 1 : -1);
        commandCounts.put(name, count);
        Logger.recordOutput("CommandsAll/LoggedCommands/" + name, count > 0);
    }

    @Override
    public void initialize() {
        log(true);
        child.initialize();
    }

    @Override
    public void execute() {
        child.execute();
    }

    @Override
    public boolean isFinished() {
        return child.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        log(false);
        child.end(interrupted);
    }
}
