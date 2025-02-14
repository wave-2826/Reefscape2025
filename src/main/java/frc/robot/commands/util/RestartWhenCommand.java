package frc.robot.commands.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** A command that runs the given command and ends with it but restarts the command if the given condition is met. */
public class RestartWhenCommand extends Command {
    private final Command command;
    private final BooleanSupplier condition;
    private boolean firstCycle = true;

    public RestartWhenCommand(Command command, BooleanSupplier condition) {
        this.command = command;
        this.condition = condition;

        CommandScheduler.getInstance().registerComposedCommands(command);
        addRequirements(command.getRequirements());
        setName("RestartWhenCommand(" + command.getName() + ")");
    }

    @Override
    public void initialize() {
        command.initialize();
        firstCycle = true;
    }

    @Override
    public void execute() {
        command.execute();
        if(condition.getAsBoolean() && !firstCycle) {
            firstCycle = false;
            command.end(true);
            command.initialize();
        }
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }
}
