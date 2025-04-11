package frc.robot.util.sim.adapters;

import org.ironmaple.simulation.SimulatedArena;

import frc.robot.util.sim.SimulationAdapter;
import frc.robot.util.sim.simField.SimulatedReefscapeArena;

public class CustomFieldSimAdapter implements SimulationAdapter {
    @Override
    public void preInit() {
        SimulatedArena.overrideInstance(new SimulatedReefscapeArena());
    }
}
