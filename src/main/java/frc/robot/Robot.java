package frc.robot;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableSparkPID;
import frc.robot.util.NTClientLogger;
import frc.robot.util.Pn532;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import au.grapplerobotics.CanBridge;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends LoggedRobot {
    /**
     * The timeout for the robot to print a loop overrun message, in seconds. We do some reflection trickery to adjust
     * the private field in IterativeRobotBase.
     */
    private static final double loopOverrunWarningTimeout = 0.2;

    private Command autonomousCommand;
    private RobotContainer robotContainer;

    public Robot() {
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch(BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        Logger.recordMetadata("BatteryID", getBatteryID());

        // Adjust the loop overrun warning timeout; taken from 6328's code.
        // This is obviously a bit hacky, but we log our loop times and consistently watch them,
        // so the loop overrun messages just become noise and make it hard to see real issues in
        // the console. Therefore, we increase the timeout to 0.2 seconds to reduce the noise.
        try {
            // The field is private by default, so we need to make it accessible and
            // use reflection to access its value. This is a bit hacky, but it works.
            Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
            watchdogField.setAccessible(true);
            Watchdog watchdog = (Watchdog) watchdogField.get(this);
            watchdog.setTimeout(loopOverrunWarningTimeout);
        } catch(Exception e) {
            DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
        }

        // Set up data receivers & replay source
        switch(Constants.currentMode) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case SIM:
                // Running a physics simulator, log to NT
                if(Constants.logInSimulation) Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
                break;
        }

        // Initialize URCL
        Logger.registerURCL(URCL.startExternal());

        // Start AdvantageKit logger
        Logger.start();

        DriverStation.silenceJoystickConnectionWarning(true);

        // Log active commands. Also taken from 6328's code.
        Map<String, Integer> commandCounts = new HashMap<>();
        BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
            String name = command.getName();
            int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
            commandCounts.put(name, count);
            // We currently don't log unique commands unless we're in replay since it isn't
            // very helpful and adds a lot of noise to the logs.
            if(Constants.currentMode == Constants.Mode.REPLAY) {
                Logger.recordOutput("CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
            }

            Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
        CommandScheduler.getInstance()
            .onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
        CommandScheduler.getInstance().onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
        CommandScheduler.getInstance()
            .onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));

        // This most likely isn't a good idea, but we experience so many power issues
        // that we reduce the RoboRIO brownout voltage. The RoboRIO 2 originally had a
        // brownout voltage of 6.25 before it was increased, so we're comfortable
        // setting it to 6.0. This hasn't caused issues in the past, but it's obviously
        // not an ideal solution.
        RobotController.setBrownoutVoltage(6.0);

        // For GrappleHook
        if(Constants.currentMode == Constants.Mode.REAL && Constants.tuningMode) {
            CanBridge.runTCP();
        }

        // Configure the driver station in simulation
        if(Constants.currentMode == Constants.Mode.SIM) {
            DriverStationSim.setAllianceStationId(AllianceStationID.Blue3);
            DriverStationSim.notifyNewData();
        }

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

        robotContainer.resetSimulatedRobot();

        if(Constants.currentMode == Constants.Mode.REAL && Constants.useSuperDangerousRTThreadPriority) {
            // Switch the thread to high priority to improve loop timing.
            // This is a dangerous operation! Read the comment on useSuperDangerousRTThreadPriority and understand what this does before using it anywhere.
            Threads.setCurrentThreadPriority(true, 10);
        }
    }

    private String getBatteryID() {
        Pn532 reader = new Pn532();
        String result = reader.readAsciiBytes();
        reader.close();

        if(result == null) { return "Unknown"; }
        return result;
    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic() {
        LoggedTracer.reset();

        // Switch thread to high priority to improve loop timing
        if(!Constants.useSuperDangerousRTThreadPriority) Threads.setCurrentThreadPriority(true, 99);

        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.
        CommandScheduler.getInstance().run();

        // Return to normal thread priority
        if(!Constants.useSuperDangerousRTThreadPriority) Threads.setCurrentThreadPriority(false, 10);

        LoggedTracer.record("Commands");

        // Tunable spark PIDs
        LoggedTunableSparkPID.periodic();

        LoggedTracer.record("Tunables");

        // Alert-related updates
        RioAlerts.getInstance().update();
        Controls.getInstance().update();

        LoggedTracer.record("Alerts");

        // Miscellaneous logging
        NTClientLogger.log();

        LoggedTracer.record("Logging");
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        robotContainer.resetSimulationField();
    }

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();
        if(autonomousCommand != null) autonomousCommand.schedule();
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if(autonomousCommand != null) autonomousCommand.cancel();
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();

        robotContainer.resetSimulatedRobot();
    }

    @Override
    public void teleopPeriodic() {
        // Stop the default "override me" alert
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        robotContainer.updateSimulation();
    }
}
