package frc.robot.util;

import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SignalsConfig;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;

public class SparkUtil {
    /** Stores whether any error was has been detected by other utility methods. */
    public static boolean sparkStickyFault = false;

    /** A SignalsConfig with all status signals set to reasonable speeds. */
    public static SignalsConfig defaultSignals = new SignalsConfig()
    // @formatter:off
        .faultsAlwaysOn(true)
        .warningsAlwaysOn(true)
        .iAccumulationAlwaysOn(false)
        .analogVoltageAlwaysOn(false)
        .analogPositionAlwaysOn(false)
        .analogVelocityAlwaysOn(false)
        .primaryEncoderPositionAlwaysOn(false)
        .primaryEncoderVelocityAlwaysOn(false)
        .absoluteEncoderPositionAlwaysOn(false)
        .absoluteEncoderVelocityAlwaysOn(false)
        .externalOrAltEncoderPositionAlwaysOn(false)
        .externalOrAltEncoderVelocityAlwaysOn(false)
        
        .faultsPeriodMs(250)
        .limitsPeriodMs(20)
        .warningsPeriodMs(250)
        .busVoltagePeriodMs(20)
        .analogVoltagePeriodMs(250)
        .appliedOutputPeriodMs(20)
        .iAccumulationPeriodMs(250)
        .outputCurrentPeriodMs(20)
        .analogPositionPeriodMs(250)
        .analogVelocityPeriodMs(250)
        .motorTemperaturePeriodMs(250)
        .primaryEncoderPositionPeriodMs(100)
        .primaryEncoderVelocityPeriodMs(100)
        .absoluteEncoderPositionPeriodMs(250)
        .absoluteEncoderVelocityPeriodMs(250)
        .externalOrAltEncoderPosition(250)
        .externalOrAltEncoderVelocity(250);
    // @formatter:on

    /** Processes a value from a Spark only if the value is valid. */
    public static void ifOk(SparkBase spark, DoubleSupplier supplier, DoubleConsumer consumer) {
        double value = supplier.getAsDouble();
        if(spark.getLastError() == REVLibError.kOk) consumer.accept(value);
        else sparkStickyFault = true;
    }

    /** Processes a value from a Spark only if the value is valid. */
    public static void ifOk(SparkBase spark, BooleanSupplier supplier, BooleanConsumer consumer) {
        boolean value = supplier.getAsBoolean();
        if(spark.getLastError() == REVLibError.kOk) consumer.accept(value);
        else sparkStickyFault = true;
    }

    /** Processes a value from a Spark only if the value is valid. */
    public static <T> void ifOk(SparkBase spark, DoubleSupplier[] suppliers, Consumer<double[]> consumer) {
        double[] values = new double[suppliers.length];
        for(int i = 0; i < suppliers.length; i++) {
            values[i] = suppliers[i].getAsDouble();
            if(spark.getLastError() != REVLibError.kOk) {
                sparkStickyFault = true;
                return;
            }
        }
        consumer.accept(values);
    }

    /** Processes values from multiple Sparks only if the values are valid. */
    public static void ifOk(SparkBase[] sparks, DoubleSupplier[] suppliers, Consumer<double[]> consumer) {
        double[] values = new double[suppliers.length];
        for(int i = 0; i < suppliers.length; i++) {
            values[i] = suppliers[i].getAsDouble();
            if(sparks[i].getLastError() != REVLibError.kOk) {
                sparkStickyFault = true;
                return;
            }
        }
        consumer.accept(values);
    }

    /** Attempts to run the command until no error is produced. */
    public static void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
        for(int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if(error == REVLibError.kOk) break;
            else sparkStickyFault = true;
        }
    }

    private static class SparkFaultAlerts {
        private final SparkBase spark;
        private final Alert[] faultAlerts;
        private final Alert[] warningAlerts;

        public SparkFaultAlerts(SparkBase spark, String name) {
            this.spark = spark;
            this.faultAlerts = new Alert[] {
                // public final boolean other;
                // public final boolean motorType;
                // public final boolean sensor;
                // public final boolean can;
                // public final boolean temperature;
                // public final boolean gateDriver;
                // public final boolean escEeprom;
                // public final boolean firmware;

                new Alert(name + " other fault", Alert.AlertType.kError),
                new Alert(name + " motor type fault", Alert.AlertType.kError),
                new Alert(name + " sensor fault", Alert.AlertType.kError),
                new Alert(name + " can fault", Alert.AlertType.kError),
                new Alert(name + " temperature fault", Alert.AlertType.kError),
                new Alert(name + " gate driver fault", Alert.AlertType.kError),
                new Alert(name + " esc eeprom fault", Alert.AlertType.kError),
                new Alert(name + " firmware fault", Alert.AlertType.kError)
            };
            this.warningAlerts = new Alert[] {
                // public final boolean brownout;
                // public final boolean overcurrent;
                // public final boolean escEeprom;
                // public final boolean extEeprom;
                // public final boolean sensor;
                // public final boolean stall;
                // public final boolean hasReset;
                // public final boolean other;

                null, //
                new Alert(name + " brownout warning", Alert.AlertType.kWarning),
                new Alert(name + " overcurrent warning", Alert.AlertType.kWarning),
                new Alert(name + " esc eeprom warning", Alert.AlertType.kWarning),
                new Alert(name + " ext eeprom warning", Alert.AlertType.kWarning),
                new Alert(name + " sensor warning", Alert.AlertType.kWarning),
                new Alert(name + " stall warning", Alert.AlertType.kWarning), //
                null, // new Alert(name + " has reset warning", Alert.AlertType.kWarning),
                new Alert(name + " other warning", Alert.AlertType.kWarning)
            };
        }

        public void updateAlerts() {
            int faults = spark.getFaults().rawBits;
            int warnings = spark.getWarnings().rawBits;
            for(int i = 0; i < faultAlerts.length; i++) {
                if(faultAlerts[i] != null) faultAlerts[i].set(((faults >> i) & 1) == 1);
            }
            for(int i = 0; i < warningAlerts.length; i++) {
                if(warningAlerts[i] != null) warningAlerts[i].set(((warnings >> i) & 1) == 1);
            }
        }
    }

    private static final List<SparkFaultAlerts> sparkFaultAlerts = new ArrayList<>();

    /** Registers a spark to show alerts if there are any active motor controller faults or warnings. */
    public static void registerSparkFaultAlerts(SparkBase spark, String name) {
        sparkFaultAlerts.add(new SparkFaultAlerts(spark, name));
    }

    private static Timer updateFaultsTimer = new Timer();
    static {
        updateFaultsTimer.start();
    }

    /** Updates all registered spark fault alerts. This should probably be called relatively infrequently. */
    public static void updateSparkFaultAlerts() {
        if(updateFaultsTimer.advanceIfElapsed(0.25)) {
            for(SparkFaultAlerts sparkFaultAlert : sparkFaultAlerts) {
                sparkFaultAlert.updateAlerts();
            }
        }
    }

    public static double[] getSimulationOdometryTimestamps() {
        final double[] odometryTimeStamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
        for(int i = 0; i < odometryTimeStamps.length; i++) {
            odometryTimeStamps[i] = Timer.getFPGATimestamp() - 0.02 + i * SimulatedArena.getSimulationDt().in(Seconds);
        }
        return odometryTimeStamps;
    }
}
