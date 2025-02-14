package frc.robot.util.sim;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.interfaces.LaserCanInterface;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDeviceJNI;
import edu.wpi.first.hal.SimEnum;
import edu.wpi.first.hal.SimInt;

/**
 * Simulated LaserCAN sensor. This class is used to simulate the LaserCAN sensor in a simulation environment.
 */
public class LaserCanSim implements LaserCanInterface {
    private final SimDevice simDevice;
    private final SimEnum m_status;
    private final SimInt m_distancemm;
    private final SimInt m_ambient;
    private final SimBoolean m_islong;
    private final SimEnum m_timingBudget;
    private final SimInt m_roiX, m_roiY, m_roiW, m_roiH;

    private Measurement latestMeasurement;

    /**
     * Create a new LaserCAN sensor.
     *
     * @param can_id The CAN ID for the LaserCAN sensor. This ID is unique, and set in GrappleHook. Note: one ID should
     *            be mapped to only one sensor, or else measurements will conflict.
     */
    public LaserCanSim(int can_id) {
        latestMeasurement = new Measurement(0, 0, 0, false, TimingBudget.TIMING_BUDGET_20MS.asMilliseconds(),
            new RegionOfInterest(0, 0, 16, 16));

        simDevice = new SimDevice(SimDeviceJNI.createSimDevice("LaserCAN [" + can_id + "]"));
        m_distancemm = simDevice.createInt("distance_mm", Direction.kBidir, latestMeasurement.distance_mm);
        m_ambient = simDevice.createInt("ambient", Direction.kBidir, latestMeasurement.ambient);
        m_islong = simDevice.createBoolean("is_long", Direction.kOutput, latestMeasurement.is_long);
        m_timingBudget = simDevice.createEnum("Timing Budget", Direction.kOutput, new String[] {
            "TIMING_BUDGET_20MS", "TIMING_BUDGET_33MS", "TIMING_BUDGET_50MS", "TIMING_BUDGET_100MS"
        }, 0);
        m_status = simDevice.createEnum("status", Direction.kBidir, new String[] {
            "LASERCAN_STATUS_VALID_MEASUREMENT", "LASERCAN_STATUS_NOISE_ISSUE", "LASERCAN_STATUS_WEAK_SIGNAL",
            "LASERCAN_STATUS_OUT_OF_BOUNDS", "LASERCAN_STATUS_WRAPAROUND"
        }, 0);
        m_roiX = simDevice.createInt("ROI X", Direction.kOutput, latestMeasurement.roi.x);
        m_roiY = simDevice.createInt("ROI Y", Direction.kOutput, latestMeasurement.roi.y);
        m_roiW = simDevice.createInt("ROI W", Direction.kOutput, latestMeasurement.roi.w);
        m_roiH = simDevice.createInt("ROI H", Direction.kOutput, latestMeasurement.roi.h);
    }

    /**
     * Get the most recent measurement from the sensor, if available. May return null.
     */
    @Override
    public Measurement getMeasurement() {
        latestMeasurement.ambient = m_ambient.get();
        latestMeasurement.distance_mm = m_distancemm.get();
        latestMeasurement.is_long = m_islong.get();
        latestMeasurement.status = m_status.get();
        latestMeasurement.roi.x = m_roiX.get();
        latestMeasurement.roi.y = m_roiY.get();
        latestMeasurement.roi.w = m_roiW.get();
        latestMeasurement.roi.h = m_roiH.get();
        return latestMeasurement;
    }

    /**
     * Set the ranging mode for the sensor.
     *
     * @param mode
     * @see RangingMode
     */
    @Override
    public void setRangingMode(RangingMode mode) throws ConfigurationFailedException {
        latestMeasurement.is_long = mode == RangingMode.LONG;
        m_islong.set(mode == RangingMode.LONG);
    }

    /**
     * Set the timing budget for the sensor.
     *
     * @param budget
     * @see TimingBudget
     */
    @Override
    public void setTimingBudget(TimingBudget budget) throws ConfigurationFailedException {
        latestMeasurement.budget_ms = budget.asMilliseconds();
        m_timingBudget.set(budget.ordinal());
    }

    /**
     * Set the region of interest for the sensor.
     *
     * @param roi
     * @see RegionOfInterest
     */
    @Override
    public void setRegionOfInterest(RegionOfInterest roi) throws ConfigurationFailedException {
        latestMeasurement.roi = roi;
        m_roiX.set(roi.x);
        m_roiY.set(roi.y);
        m_roiW.set(roi.w);
        m_roiH.set(roi.h);

    }

    /**
     * Set the simulated sensor's latest measurement. Only includes the measurement parameters, not including ranging
     * mode, timing budget, or ROI.
     */
    public void setMeasurement(int status, int distance_mm, int ambient) {
        latestMeasurement.status = status;
        latestMeasurement.distance_mm = distance_mm;
        latestMeasurement.ambient = ambient;
        m_status.set(status);
        m_distancemm.set(distance_mm);
        m_ambient.set(ambient);
    }
}
