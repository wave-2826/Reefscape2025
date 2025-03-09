package frc.robot.commands.util;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A command that runs the given command and ends with it but restarts the command if the given condition is met. */
public class RestartWhenCommand extends Command {
    private final Supplier<Command> command;
    private final BooleanSupplier condition;
    private Command currentCommand;
    private boolean firstCycle = true;

    public RestartWhenCommand(Supplier<Command> command, BooleanSupplier condition, Set<SubsystemBase> requirements) {
        this.command = command;
        this.condition = condition;
        addRequirements(requirements.toArray(new Subsystem[0]));
        setName("RestartWhenCommand");
    }

    @Override
    public void initialize() {
        currentCommand = command.get();
        currentCommand.initialize();
        CommandScheduler.getInstance().registerComposedCommands(currentCommand);
        firstCycle = true;
    }

    @Override
    public void execute() {
        currentCommand.execute();
        if(condition.getAsBoolean() && !firstCycle) {
            firstCycle = false;
            currentCommand.end(true);
            currentCommand.initialize();
        }
    }

    @Override
    public boolean isFinished() {
        return currentCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        currentCommand.end(interrupted);
    }
}
