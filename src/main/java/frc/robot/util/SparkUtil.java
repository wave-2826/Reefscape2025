package frc.robot.util;

import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SignalsConfig;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.Timer;
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

    public static double[] getSimulationOdometryTimeStamps() {
        final double[] odometryTimeStamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
        for(int i = 0; i < odometryTimeStamps.length; i++) {
            odometryTimeStamps[i] = Timer.getFPGATimestamp() - 0.02 + i * SimulatedArena.getSimulationDt().in(Seconds);
        }
        return odometryTimeStamps;
    }
}
