package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

public class DriveTuningCommands {
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec

    private static final double SLIP_START_DELAY = 1.0; // Secs
    private static final double SLIP_START_VOLTAGE = 0.4; // Volts
    private static final double SLIP_RAMP_RATE = 0.075; // Volts/Sec
    private static final double SLIP_TRAVEL_AMOUNT = Units.degreesToRadians(15); // Rad

    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    private static SysIdRoutine sysIdRoutine = null;

    private DriveTuningCommands() {
    }

    /** Adds the drive tuning commands to the auto chooser. */
    public static void addTuningCommandsToAutoChooser(Drive drive, LoggedDashboardChooser<Command> chooser) {
        // We might want to run these at a competition
        chooser.addOption("TUNING | Drive Wheel Radius Characterization", wheelRadiusCharacterization(drive));
        chooser.addOption("TUNING | Drive Slip Current Measurement", slipCurrentMeasurement(drive));

        // These only apply to when we're doing "real" tuning
        if(Constants.tuningMode) {
            chooser.addOption("TUNING | Drive Simple FF Characterization", feedforwardCharacterization(drive));

            chooser.addOption("TUNING | Drive SysId (Quasistatic Forward)",
                sysIdQuasistatic(drive, SysIdRoutine.Direction.kForward));
            chooser.addOption("TUNING | Drive SysId (Quasistatic Reverse)",
                sysIdQuasistatic(drive, SysIdRoutine.Direction.kReverse));
            chooser.addOption("TUNING | Drive SysId (Dynamic Forward)",
                sysIdDynamic(drive, SysIdRoutine.Direction.kForward));
            chooser.addOption("TUNING | Drive SysId (Dynamic Reverse)",
                sysIdDynamic(drive, SysIdRoutine.Direction.kReverse));
        }
    }

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>
     * This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization(Drive drive) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
            // Reset data
            Commands.runOnce(() -> {
                velocitySamples.clear();
                voltageSamples.clear();
            }),

            // Allow modules to orient
            Commands.run(() -> {
                drive.runCharacterization(0.0);
            }, drive).withTimeout(FF_START_DELAY),

            // Start timer
            Commands.runOnce(timer::restart),

            // Accelerate and gather data
            Commands.run(() -> {
                double voltage = timer.get() * FF_RAMP_RATE;
                drive.runCharacterization(voltage);
                velocitySamples.add(drive.getFFCharacterizationVelocity());
                voltageSamples.add(voltage);
            }, drive).finallyDo(() -> { // When cancelled, calculate and print results
                int n = velocitySamples.size();
                double sumX = 0.0;
                double sumY = 0.0;
                double sumXY = 0.0;
                double sumX2 = 0.0;
                for(int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                }
                double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                NumberFormat formatter = new DecimalFormat("#0.00000");
                System.out.println("********** Drive FF Characterization Results **********");
                System.out.println("\tkS: " + formatter.format(kS));
                System.out.println("\tkV: " + formatter.format(kV));
            }));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization(Drive drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
            // Drive control sequence
            Commands.sequence(
                // Reset acceleration limiter
                Commands.runOnce(() -> {
                    limiter.reset(0.0);
                }),

                // Turn in place, accelerating up to full speed
                Commands.run(() -> {
                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                }, drive)),

            // Measurement sequence
            Commands.sequence(
                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),

                // Record starting measurement
                Commands.runOnce(() -> {
                    state.positions = drive.getWheelRadiusCharacterizationPositions();
                    state.lastAngle = drive.getRotation();
                    state.gyroDelta = 0.0;
                }),

                // Update gyro delta
                Commands.run(() -> {
                    var rotation = drive.getRotation();
                    state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                    state.lastAngle = rotation;
                })

                    // When cancelled, calculate and print results
                    .finallyDo(() -> {
                        double[] positions = drive.getWheelRadiusCharacterizationPositions();
                        double wheelDelta = 0.0;
                        for(int i = 0; i < 4; i++) wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                        double wheelRadius = (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                        NumberFormat formatter = new DecimalFormat("#0.000");
                        System.out.println("********** Wheel Radius Characterization Results **********");
                        System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                        System.out.println("\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                        System.out.println("\tWheel Radius: " + formatter.format(wheelRadius) + " meters, "
                            + formatter.format(Units.metersToInches(wheelRadius)) + " inches");
                    })));
    }

    private static class SlipCurrentState {
        double startPosition = 0.0;
    }

    /**
     * Measures the current at which the robot slips by progressively increasing the wheel voltage and measuring when
     * their velocity jumps. The robot _must_ be placed against a wall for this to work.
     */
    public static Command slipCurrentMeasurement(Drive drive) {
        List<double[]> currentSamples = new LinkedList<>();
        Timer timer = new Timer();

        SlipCurrentState state = new SlipCurrentState();

        // TODO: Increase drive current limit for slip measurement

        return Commands.sequence(
            // Reset data
            Commands.runOnce(() -> {
                currentSamples.clear();
            }),

            // Allow modules to orient
            Commands.run(() -> {
                drive.runCharacterization(0.0);
            }, drive).withTimeout(SLIP_START_DELAY),

            // Reset the start position
            Commands.runOnce(() -> state.startPosition = drive.getSlipMeasurementPosition()),

            // Start timer
            Commands.runOnce(timer::restart),

            // Accelerate and gather data
            Commands.run(() -> {
                double voltage = timer.get() * SLIP_RAMP_RATE + SLIP_START_VOLTAGE;
                drive.runCharacterization(voltage);

                currentSamples.add(drive.getSlipMeasurementCurrents());
            }, drive).until(() -> {
                double distanceTraveled = Math.abs(drive.getSlipMeasurementPosition() - state.startPosition);
                return distanceTraveled > SLIP_TRAVEL_AMOUNT;
            }),

            // Take a few samples behind when we stopped and print the result
            Commands.runOnce(() -> {
                drive.runCharacterization(0.0);

                double[] slipCurrents = currentSamples.get(currentSamples.size() - 4);
                double averageSlipCurrent = 0.0;
                for(int i = 0; i < 4; i++) averageSlipCurrent += slipCurrents[i] / 4.0;

                double slipVoltage = timer.get() * SLIP_RAMP_RATE + SLIP_START_VOLTAGE;

                System.out.println("********** Drive Slip Current Measurement Results **********");
                System.out.println("\tAverage slip Current: " + (int) Math.floor(averageSlipCurrent) + " amps");
                System.out.println("\tSlip \"Voltage\": " + slipVoltage + " volts");
                String[] moduleNames = new String[] {
                    "Front left", "Front right", "Back left", "Back right"
                };
                NumberFormat formatter = new DecimalFormat("#0.000");
                System.out.println("\tIndividual module slip currents:");
                for(int i = 0; i < 4; i++) {
                    System.out.println("\t \t" + moduleNames[i] + ": " + formatter.format(slipCurrents[i]));
                }

                // Estimate the wheel's coefficient of friction
                double motorTorque = averageSlipCurrent * DriveConstants.driveSimMotor.KtNMPerAmp;
                double totalTorqueNm = 4 * DriveConstants.driveMotorReduction * motorTorque;
                double robotMassN = DriveConstants.robotMassKg * 9.81;
                double wheelCOF = totalTorqueNm / (robotMassN * DriveConstants.wheelRadiusMeters);
                NumberFormat cofFormatter = new DecimalFormat("#0.0000");
                System.out.println("\tWheel COF: " + cofFormatter.format(wheelCOF));
            }));
    }

    /** Configures the SysId routine if it hasn't been configured yet. */
    private static void initSysId(Drive drive) {
        if(sysIdRoutine == null) {
            sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(null, null, null,
                    (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> drive.runCharacterization(voltage.in(Volts)), null, drive));
        }
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public static Command sysIdQuasistatic(Drive drive, SysIdRoutine.Direction direction) {
        initSysId(drive);

        return Commands.run(() -> drive.runCharacterization(0.0), drive).withTimeout(1.0)
            .andThen(sysIdRoutine.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public static Command sysIdDynamic(Drive drive, SysIdRoutine.Direction direction) {
        initSysId(drive);

        return Commands.run(() -> drive.runCharacterization(0.0), drive).withTimeout(1.0)
            .andThen(sysIdRoutine.dynamic(direction));
    }
}
