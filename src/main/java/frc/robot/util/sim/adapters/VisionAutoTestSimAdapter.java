package frc.robot.util.sim.adapters;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.util.SimControls;
import frc.robot.util.sim.SimulationAdapter;

public class VisionAutoTestSimAdapter implements SimulationAdapter {
    @Override
    public Command transformAutoCommand(Command command, RobotContainer robotContainer) {
        return new LoggedCommand("AutoSimAuto", Commands.sequence(//
            Commands.runOnce(() -> {
                System.out.println("Starting auto simulation");
                robotContainer.resetSimulatedRobot();
                robotContainer.resetSimulationField();
                ArmIOSim.addGamePiece();
            }), //
            new ScheduleCommand(command),
            // Emulate the human player dropping pieces
            Commands.defer(() -> {
                boolean onRight = RobotState.getInstance().isOnRightSide();
                return Commands.sequence(SimControls.dropCoralCommand(onRight, true), Commands.waitSeconds(4),
                    SimControls.dropCoralCommand(onRight, true), Commands.waitSeconds(4),
                    SimControls.dropCoralCommand(onRight, true), Commands.waitSeconds(4),
                    SimControls.dropCoralCommand(onRight, true));
            }, Set.of())//
        ));
    }
}
