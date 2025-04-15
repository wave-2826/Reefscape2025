package frc.robot.util.sim.adapters;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.util.sim.SimulationAdapter;

public class AutoMatchTimeAdapter implements SimulationAdapter {
    private static final double START_DELAY = 2.5;
    private static final double DISABLE_DELAY = 0.25;
    private static final double AUTO_TIME = 15.0;
    private static final double AUTO_TELEOP_DELAY = 3;
    private static final double TELEOP_TIME = 10000; // 135.0;

    private static final Timer simStartTimer = new Timer();
    private static final Timer timer = new Timer();

    @Override
    public void postInit() {
        simStartTimer.start();
    }

    @Override
    public void tick() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue3);

        DriverStationSim.setDsAttached(true);

        if(simStartTimer.hasElapsed(START_DELAY)) {
            simStartTimer.stop();
            timer.start();
        }
        if(simStartTimer.isRunning()) return;

        if(DriverStationSim.getTest()) {
            timer.restart();
            DriverStationSim.setTest(false);
        }

        if(timer.get() > DISABLE_DELAY && timer.get() < DISABLE_DELAY + AUTO_TIME) {
            DriverStationSim.setEnabled(true);
            DriverStationSim.setAutonomous(true);
        } else if(timer.get() > DISABLE_DELAY + AUTO_TIME + AUTO_TELEOP_DELAY
            && timer.get() < DISABLE_DELAY + AUTO_TIME + AUTO_TELEOP_DELAY + TELEOP_TIME//
        ) {
            DriverStationSim.setEnabled(true);
            DriverStationSim.setAutonomous(false);
        } else {
            DriverStationSim.setEnabled(false);
        }

        DriverStationSim.notifyNewData();
    }
}
