package frc.robot.util.sim.adapters;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.util.sim.SimulationAdapter;

public class TeleopAdapter implements SimulationAdapter {
    @Override
    public void tick() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue3);
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);

        DriverStationSim.notifyNewData();
    }
}
