package frc.robot.util;

import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.ArrayList;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

/**
 * A class to manage logged, tunable, PIDF constants for Spark motor controllers. This class allows for the PIDF
 * constants to be tunable via the robot's dashboard if the robot is in tuning mode. It allows for separate PIDF
 * constnats for the real and simulated robot.
 */
public class LoggedTunableSparkPID {
    /** A set of PID constants with tunable numbers for each. */
    private class PIDConstants {
        public LoggedTunableNumber p;
        public LoggedTunableNumber i;
        public LoggedTunableNumber d;
        public LoggedTunableNumber f;
        public ClosedLoopSlot slot;

        public PIDConstants(double p, double i, double d, double f, ClosedLoopSlot slot) {
            this.p = new LoggedTunableNumber(tunablePath + slot.toString() + "P", p);
            this.i = new LoggedTunableNumber(tunablePath + slot.toString() + "I", i);
            this.d = new LoggedTunableNumber(tunablePath + slot.toString() + "D", d);
            this.f = new LoggedTunableNumber(tunablePath + slot.toString() + "F", f);
            this.slot = slot;
        }

        public boolean hasChanged() {
            return p.hasChanged(hashCode()) || i.hasChanged(hashCode()) || d.hasChanged(hashCode())
                || f.hasChanged(hashCode());
        }
    }

    /** The set of PID slots to be used when on a real robot. */
    private ArrayList<PIDConstants> realRobotConstants;
    /** The set of PID slots to be used when on a simulated robot. */
    private ArrayList<PIDConstants> simConstants;
    /** The path to the tunable constants. */
    private String tunablePath;
    /** The list of sparks to be configured. */
    private ArrayList<SparkBase> sparks = new ArrayList<>();

    /** A list of change listeners that are run every loop iteration when in tuning mode. */
    private static ArrayList<Runnable> changeListenerRegistry = new ArrayList<>();

    /**
     * A static method to be called every loop iteration to check for changes and re-configure the sparks. Doesn't
     * unnecessarily run if not in tuning mode.
     */
    public static void periodic() {
        if(!Constants.tuningMode) { return; }
        for(Runnable r : changeListenerRegistry) {
            r.run();
        }
    }

    /**
     * Creates a new TunableSparkPID object with the given path. The path is used to create the tunable numbers for the
     * PID constants. Note tht this object will never be cleaned up once created, so it should be a constant.
     *
     * @param tunablePath The path to the tunable constants.
     */
    public LoggedTunableSparkPID(String tunablePath) {
        realRobotConstants = new ArrayList<>();
        simConstants = new ArrayList<>();
        this.tunablePath = tunablePath;

        // Register a change listener to check for changes
        changeListenerRegistry.add(() -> {
            checkChange();
        });
    }

    /**
     * Gets the closed loop configuration for the current mode. This should be used with
     * `SparkMaxConfiguration.closedLoop.apply` or `SparkFlexConfiguration.closedLoop.apply` when configuring the motor
     * controllers.
     * @return
     */
    public ClosedLoopConfig getConfig() {
        ClosedLoopConfig config = new ClosedLoopConfig();
        ArrayList<PIDConstants> constants = Constants.currentMode == Constants.Mode.REAL ? realRobotConstants
            : simConstants;
        for(PIDConstants c : constants) {
            config.pidf(c.p.get(), c.i.get(), c.d.get(), c.f.get(), c.slot);
        }
        return config;
    }

    /**
     * Registers a spark to be configured when the PID constants change. This should be called for each spark that needs
     * to be configured. Note that this doesn't immediately configure the spark.
     * @param spark
     */
    public void configureSparkOnChange(SparkBase spark) {
        if(!Constants.tuningMode) { return; } // No need to do unnecessary work if not in tuning mode
        sparks.add(spark);
    }

    /**
     * Checks if any of the PID constants have changed. If they have, it reconfigures all of the sparks that have been
     * registered. This is called every loop iteration when in tuning mode.
     */
    private void checkChange() {
        ArrayList<PIDConstants> constants = Constants.currentMode == Constants.Mode.REAL ? realRobotConstants
            : simConstants;
        for(PIDConstants c : constants) {
            if(c.hasChanged()) {
                for(SparkBase spark : sparks) {
                    boolean isFlex = spark instanceof SparkFlex;
                    SparkBaseConfig config = isFlex ? new SparkFlexConfig() : new SparkMaxConfig();
                    config.closedLoop.apply(getConfig());

                    tryUntilOk(spark, 5, () -> spark.configure(config, ResetMode.kNoResetSafeParameters,
                        PersistMode.kNoPersistParameters));
                    System.out.println("Configured " + spark.getDeviceId() + " with new PID constants");
                }
                return;
            }
        }
    }

    // A bunch of different overloads so the feedforward and slot are optional

    /**
     * Adds a new set of PID constants to the list of constants for the real robot.
     * @param p
     * @param i
     * @param d
     * @param f
     * @param slot
     */
    public LoggedTunableSparkPID addRealRobotGains(double p, double i, double d, double f, ClosedLoopSlot slot) {
        realRobotConstants.add(new PIDConstants(p, i, d, f, slot));
        return this;
    }

    /**
     * Adds a new set of PID constants to the list of constants for the simulated robot.
     * @param p
     * @param i
     * @param d
     * @param f
     * @param slot
     */
    public LoggedTunableSparkPID addSimGains(double p, double i, double d, double f, ClosedLoopSlot slot) {
        simConstants.add(new PIDConstants(p, i, d, f, slot));
        return this;
    }

    /**
     * Adds a new set of PID constants to the list of constants for the real robot. This defaults to slot 0.
     * @param p
     * @param i
     * @param d
     */
    public LoggedTunableSparkPID addRealRobotGains(double p, double i, double d, double f) {
        return addRealRobotGains(p, i, d, f, ClosedLoopSlot.kSlot0);
    }

    /**
     * Adds a new set of PID constants to the list of constants for the simulated robot. This defaults to slot 0.
     * @param p
     * @param i
     * @param d
     */
    public LoggedTunableSparkPID addSimGains(double p, double i, double d, double f) {
        return addSimGains(p, i, d, f, ClosedLoopSlot.kSlot0);
    }

    /**
     * Adds a new set of PID constants to the list of constants for the real robot.
     * @param p
     * @param i
     * @param d
     */
    public LoggedTunableSparkPID addRealRobotGains(double p, double i, double d, ClosedLoopSlot slot) {
        return addRealRobotGains(p, i, d, 0, slot);
    }

    /**
     * Adds a new set of PID constants to the list of constants for the simulated robot.
     * @param p
     * @param i
     * @param d
     */
    public LoggedTunableSparkPID addSimGains(double p, double i, double d, ClosedLoopSlot slot) {
        return addSimGains(p, i, d, 0, slot);
    }

    /**
     * Adds a new set of PID constants to the list of constants for the real robot. This defaults to slot 0.
     * @param p
     * @param i
     * @param d
     */
    public LoggedTunableSparkPID addRealRobotGains(double p, double i, double d) {
        return addRealRobotGains(p, i, d, 0, ClosedLoopSlot.kSlot0);
    }

    /**
     * Adds a new set of PID constants to the list of constants for the simulated robot. This defaults to slot 0.
     * @param p
     * @param i
     * @param d
     */
    public LoggedTunableSparkPID addSimGains(double p, double i, double d) {
        return addSimGains(p, i, d, 0, ClosedLoopSlot.kSlot0);
    }
}
