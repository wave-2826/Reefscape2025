package frc.robot.util.sim.adapters;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.util.sim.SimulationAdapter;

public class AutoMatchTimeAdapter implements SimulationAdapter {
    private static final double START_DELAY = 2.5;
    private static final double AUTO_TIME = 15.0;
    private static final double AUTO_TELEOP_DELAY = 3;
    private static final double TELEOP_TIME = 135.0;

    @Override
    public void tick() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue3);

        DriverStationSim.setDsAttached(true);

        if(Timer.getTimestamp() > START_DELAY && Timer.getTimestamp() < START_DELAY + AUTO_TIME) {
            DriverStationSim.setEnabled(true);
            DriverStationSim.setAutonomous(true);
        } else if(Timer.getTimestamp() > START_DELAY + AUTO_TIME + AUTO_TELEOP_DELAY
            && Timer.getTimestamp() < START_DELAY + AUTO_TIME + AUTO_TELEOP_DELAY + TELEOP_TIME//
        ) {
            DriverStationSim.setEnabled(true);
            DriverStationSim.setAutonomous(false);
        } else {
            DriverStationSim.setEnabled(false);
        }

        DriverStationSim.notifyNewData();
    }
}
