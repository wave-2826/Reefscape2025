package frc.robot.subsystems.leds;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;
import java.util.TreeSet;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.LoggedTracer;
import frc.robot.util.RioAlerts;

// TODO: Alpha compositing with custom RGBA color class? (not that we care _that_ much, I suppose...)

/**
 * The LED subsystem. Manages the LED states. This is massively overengineered, but I like LEDs ;)
 */
public class LEDs extends SubsystemBase {
    /**
     * We skip updating LEDs until this many full periodic loop cycles have passed. This prevents some weird behavior.
     */
    private static final int SKIP_FIRST_LOOP_CYCLES = 10;
    /**
     * The duration of the autonomous start animation, in seconds.
     */
    private static final double AUTONOMOUS_START_ANIMATION_DURATION = 1.0;
    /**
     * The duration of the hit wall animation, in seconds.
     */
    private static final double HIT_WALL_ANIMATION_DURATION = 0.4;

    /** The WAVE Blue color. */
    private static final Color WAVE_BLUE = new Color("#00A0C3");
    /** The WAVE Green color. */
    private static final Color WAVE_GREEN = new Color("#45bf4d");

    /** The maximum speed multiplier for when the robot is going its maximum speed. */
    private static final double maximumSpeedMultiplier = 2.5;

    /**
     * The current LED colors.
     */
    private Color[] colors = new Color[LEDConstants.ledCount];
    /**
     * The active compositing mode. Set while rendering the LEDs.
     */
    private LEDCompositingMode activeCompositingMode = LEDCompositingMode.Exclusive;

    private LEDIO io;
    /**
     * The active LED states. We use a TreeSet of LEDState enums because it will automatically sort the states by their
     * priority order. Then, we display the highest priority state that is active.
     */
    private TreeSet<LEDState> activeStates = new TreeSet<LEDState>();
    /**
     * The time that each state was last started. Used to fade temporary states. NOTE: This isn't in the same time base
     * as the LED time; it uses Timer.getTimestamp() so it's not affected by the robot speed multiplier.
     */
    private HashMap<LEDState, Double> stateStartTimes = new HashMap<LEDState, Double>();

    /**
     * The number of loop cycles that have passed.
     */
    private int loopCycles = 0;

    /**
     * A notifier that runs the animation until the robot code is fully started. This allows us to display the LEDs as
     * soon as possible.
     */
    private final Notifier initialUpdateNotifier;
    /**
     * If initial updates are currently running.
     */
    private boolean initialUpdatesRunning = true;

    /**
     * The time of the last update.
     */
    private double lastTime = Timer.getTimestamp();
    /**
     * The time used for the LEDs, in seconds (but scaled by the drivetrain speed). This isn't necessarily monotonically
     * increasing.
     */
    private double time = 0;

    /**
     * A compositing mode for the LED state. Used to allow us to layer multiple LED states on top of each other.
     */
    public static enum LEDCompositingMode {
        /** The LED state is added on top of the previous state. */
        Additive,
        /** The LED state is multiplied with the previous state. */
        Multiplicative,
        /** The value of the new state is used to blend. */
        Value,
        /** This state is layered on top of all previous states. The below states aren't even rendered. */
        Exclusive
    }

    /**
     * The set of possible LED states. Each state has a lambda function that accepts the LED subsystem.
     *
     * The order of the states is their priority order. The first state in the enum is the highest priority, and the
     * last state is the lowest priority.
     */
    public static enum LEDState {
        EStopped((leds) -> leds.pulse(Color.kBlack, Color.kRed, 2.0)), // Active when the robot is E-stopped

        BatteryLow((leds) -> leds.flash(Color.kOrangeRed, Color.kBlack, 0.3, 0.05, 0.3, 5.0), LEDCompositingMode.Value), // Active when the robot battery is low

        AutonomousStart(LEDs::autonomousStart, LEDCompositingMode.Additive), // Active at the start of autonomous

        AutoScoring((leds) -> leds.rainbow()), //
        AutoScoreReady((leds) -> leds.pulse(Color.kPurple, Color.kPink, 0.3)), //

        HitWall(LEDs::hitWall, LEDCompositingMode.Value), // Active when the robot hits a wall

        Disabled((leds) -> leds.gradient(leds.allianceDark(), leds.allianceLight(), 5.0)), // Active when the robot is disabled

        Intaking((leds) -> leds.pulse(Color.kBlack, Color.kGreen, 0.5)), //
        PieceReadyForArm((leds) -> leds.pulse(Color.kBlack, Color.kYellow, 0.5)), //

        Teleop((leds) -> leds.gradient(leds.allianceDark(), leds.allianceLight(), 2.0)), // Active when the robot is in teleop
        Test((leds) -> leds.gradient(Color.kYellow, Color.kOrange, 5.0)), // Active when the robot is in test mode
        Autonomous((leds) -> leds.gradient(WAVE_BLUE, WAVE_GREEN, 1.5)), // Active when the robot is in autonomous

        Default((leds) -> leds.gradient(WAVE_BLUE, Color.kWhite, 2.0)); // Active when no other state is active

        private final LEDStateFunction function;
        private final LEDCompositingMode compositingMode;

        private LEDState(LEDStateFunction function) {
            this(function, LEDCompositingMode.Exclusive);
        }

        private LEDState(LEDStateFunction function, LEDCompositingMode compositingMode) {
            this.function = function;
            this.compositingMode = compositingMode;
        }
    }

    /**
     * The function type for LED states.
     */
    @FunctionalInterface
    private static interface LEDStateFunction {
        public void run(LEDs leds);
    }

    public LEDs(LEDIO io) {
        this.io = io;

        enableState(LEDState.Default);

        initialUpdateNotifier = new Notifier(() -> {
            synchronized(this) {
                this.time += 1. / 50.;
                this.gradient(Color.kWhite, Color.kGray, 1.0);
                this.io.pushLEDs(this.colors);
            }
        });
        initialUpdateNotifier.setName("LED Initial Updates");
        initialUpdateNotifier.startPeriodic(1. / 50.);

        registerLEDTriggers();
    }

    /**
     * Binds a state to a trigger.
     */
    private void bindStateToTrigger(LEDState state, Trigger trigger) {
        trigger.onFalse(disableStateCommand(state)).onTrue(enableStateCommand(state));
    }

    /**
     * Registers the LED triggers.
     */
    private void registerLEDTriggers() {
        var rioAlerts = RioAlerts.getInstance();
        Trigger lowBatteryTrigger = new Trigger(rioAlerts::batteryLow);
        Trigger eStoppedTrigger = new Trigger(DriverStation::isEStopped);

        // RobotModeTriggers exists, but some of them don't respect disabled mode? This seems cleaner anyway.
        Trigger autonomousTrigger = new Trigger(DriverStation::isAutonomousEnabled);
        Trigger disabledTrigger = new Trigger(DriverStation::isDisabled);
        Trigger teleopTrigger = new Trigger(DriverStation::isTeleopEnabled);
        Trigger testTrigger = new Trigger(DriverStation::isTestEnabled);

        bindStateToTrigger(LEDState.EStopped, eStoppedTrigger);
        bindStateToTrigger(LEDState.BatteryLow, lowBatteryTrigger);

        bindStateToTrigger(LEDState.Autonomous, autonomousTrigger);
        bindStateToTrigger(LEDState.Disabled, disabledTrigger);
        bindStateToTrigger(LEDState.Teleop, teleopTrigger);
        bindStateToTrigger(LEDState.Test, testTrigger);

        autonomousTrigger.onTrue(new SequentialCommandGroup(enableStateCommand(LEDState.AutonomousStart),
            Commands.waitSeconds(AUTONOMOUS_START_ANIMATION_DURATION), disableStateCommand(LEDState.AutonomousStart)));

        Drive.setAbruptStopCommand(new SequentialCommandGroup(enableStateCommand(LEDState.HitWall),
            Commands.waitSeconds(HIT_WALL_ANIMATION_DURATION), disableStateCommand(LEDState.HitWall)));

        // Start the disabled state
        enableState(LEDState.Disabled);
    }

    public void registerIntakeStates(Intake intake) {
        bindStateToTrigger(LEDState.Intaking, new Trigger(intake::intakeSensorTriggered));
        bindStateToTrigger(LEDState.PieceReadyForArm, new Trigger(intake::pieceWaitingForArm));
    }

    /**
     * Gets the time since the LED state was last changed.
     */
    private double timeSinceStart(LEDState state) {
        return Timer.getTimestamp() - stateStartTimes.getOrDefault(state, 0.);
    }

    /**
     * Enables the given LED state. States are automatically prioritized.
     */
    public void enableState(LEDState state) {
        if(!activeStates.contains(state)) {
            activeStates.add(state);
            stateStartTimes.put(state, Timer.getTimestamp());
        }
    }

    /**
     * Disables the given LED state. States are automatically prioritized.
     */
    public void disableState(LEDState state) {
        activeStates.remove(state);
    }

    /**
     * Creates a command that enables the given state.
     * @return
     */
    public Command enableStateCommand(LEDState state) {
        return new InstantCommand(() -> enableState(state)).ignoringDisable(true);
    }

    /**
     * Creates a command that disables the given state.
     */
    public Command disableStateCommand(LEDState state) {
        return new InstantCommand(() -> disableState(state)).ignoringDisable(true);
    }

    /**
     * Creates a command that runs the given state while the command runs.
     */
    public Command runStateCommand(LEDState state) {
        return Commands.startEnd(() -> enableState(state), () -> disableState(state)).ignoringDisable(true);
    }

    /**
     * Gets the dark alliance color.
     */
    private Color allianceDark() {
        if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return Color.kBlue;
        } else {
            return Color.kRed;
        }
    }

    /**
     * Gets the light alliance color.
     */
    private Color allianceLight() {
        if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return Color.kDarkBlue;
        } else {
            return Color.kDarkRed;
        }
    }

    /**
     * Sets if the given LED state is active. States are automatically prioritized.
     */
    public void setStateActive(LEDState state, boolean active) {
        if(active) {
            enableState(state);
        } else {
            disableState(state);
        }
    }

    /**
     * A solid color effect.
     */
    public void solid(Color color) {
        for(int i = 0; i < LEDConstants.ledCount; i++) {
            setLEDColor(i, color);
        }
    }

    /**
     * A moving rainbow effect.
     */
    public void rainbow() {
        for(int i = 0; i < LEDConstants.ledCount; i++) {
            double t = (time + i / (double) LEDConstants.ledCount) % 1.;
            setLEDColor(i, Color.fromHSV((int) (t * 255), 255, 255));
        }
    }

    /**
     * A flashing effect. Periods must be an even number of values, where every pair of values is a period of on and off
     * times in seconds.
     * @param color
     * @param periods
     */
    public void flash(Color colorOn, Color colorOff, double... periods) {
        double totalLength = 0;
        for(double period : periods) {
            totalLength += period;
        }

        double time = this.time % totalLength;
        double factor = 0;

        double accumulatedTime = 0;
        for(int i = 0; i < periods.length; i += 2) {
            double periodOn = periods[i];
            double periodOff = periods[i + 1];

            if(time < accumulatedTime + periodOn) {
                factor = (time - accumulatedTime) / periodOn;
                break;
            } else if(time < accumulatedTime + periodOn + periodOff) {
                factor = 1.;
                break;
            }

            accumulatedTime += periodOn + periodOff;
        }

        solid(Color.lerpRGB(colorOn, colorOff, factor));
    }

    /**
     * A pulsing color effect.
     */
    public void pulse(Color color1, Color color2, double period) {
        // We offset the period slightly based on how far the pixels are
        // from the center of the strip to create a wave effect
        for(int i = 0; i < LEDConstants.ledCount; i++) {
            double offset = (i - LEDConstants.ledCount / 2) / (double) LEDConstants.ledCount;
            double brightness = 0.5 + 0.5 * Math.sin(2 * Math.PI * time / period + offset);
            setLEDColor(i, Color.lerpRGB(color1, color2, brightness));
        }
    }

    /**
     * A rotating circular gradient effect.
     */
    public void gradient(Color color1, Color color2, double period) {
        for(int i = 0; i < LEDConstants.ledCount; i++) {
            double t = 0.5 + 0.5 * Math.sin(2 * Math.PI * (time / period + i / (double) LEDConstants.ledCount));
            setLEDColor(i, Color.lerpRGB(color1, color2, t));
        }
    }

    /**
     * The "static" effect we use when the robot hits a wall.
     */
    public static void hitWall(LEDs leds) {
        double fade = Math.min(1., leds.timeSinceStart(LEDState.HitWall) / HIT_WALL_ANIMATION_DURATION);
        var random = new Random();
        for(int i = 0; i < LEDConstants.ledCount; i++) {
            double t = random.nextDouble();
            leds.setLEDColor(i, Color.lerpRGB(Color.lerpRGB(WAVE_BLUE, Color.kWhite, t), Color.kBlack, fade));
        }
    }

    /**
     * The fade effect we use at the start of autonomous.
     */
    public static void autonomousStart(LEDs leds) {
        double fade = Math.min(1., leds.timeSinceStart(LEDState.AutonomousStart) / AUTONOMOUS_START_ANIMATION_DURATION);
        leds.solid(Color.lerpRGB(Color.kLimeGreen, Color.kBlack, fade));
    }

    /**
     * Sets the color of an LED, adjusting for the compositing mode.
     * @param index The index of the LED.
     * @param color The color to set.
     */
    private void setLEDColor(int index, Color color) {
        switch(activeCompositingMode) {
            case Additive: {
                Color oldColor = colors[index];
                colors[index] = new Color(Math.min(1., oldColor.red + color.red),
                    Math.min(1., oldColor.green + color.green), Math.min(1., oldColor.blue + color.blue));
                break;
            }
            case Multiplicative: {
                Color oldColor = colors[index];
                colors[index] = new Color(oldColor.red * color.red, oldColor.green * color.green,
                    oldColor.blue * color.blue);
                break;
            }
            case Value: {
                Color oldColor = colors[index];
                double value = Math.min(1., color.red + color.green + color.blue);
                colors[index] = Color.lerpRGB(oldColor, color, value);
                break;
            }
            case Exclusive: {
                colors[index] = color;
                break;
            }
        }
    }

    /**
     * Updates the LEDs. Runs regardless of the robot mode.
     */
    @Override
    public void periodic() {
        loopCycles++;
        if(loopCycles < SKIP_FIRST_LOOP_CYCLES) { return; }

        if(initialUpdatesRunning) {
            initialUpdateNotifier.stop();
            initialUpdatesRunning = false;
        }

        // Update the time based on the delta and robot speed
        double robotSpeedMultiplier = 1. + RobotState.getInstance().getRobotLinearVelocity()
            / DriveConstants.maxSpeedMetersPerSec * (maximumSpeedMultiplier - 1.);

        double currentTime = Timer.getTimestamp();
        time += (currentTime - lastTime) * robotSpeedMultiplier;
        lastTime = currentTime;

        ArrayList<String> activeStateNames = new ArrayList<>();

        if(!activeStates.isEmpty()) {
            // To render the LEDs with compositing, we need to find the highest-priority exclusive
            // state and render upward from that. That way, we avoid unnecessary work that will
            // be overwritten by the exclusive state.
            LEDState[] activeStatesArray = activeStates.toArray(new LEDState[0]);

            int exclusiveStateIndex = activeStatesArray.length;
            for(int i = 0; i < activeStatesArray.length - 1; i++) {
                if(activeStatesArray[i].compositingMode == LEDCompositingMode.Exclusive) {
                    exclusiveStateIndex = i;
                    break;
                }
            }

            for(int i = exclusiveStateIndex; i >= 0; i--) {
                activeCompositingMode = activeStatesArray[i].compositingMode;
                activeStatesArray[i].function.run(this);
                activeStateNames.add(activeStatesArray[i].name());
            }
        }

        Logger.recordOutput("LEDs/ActiveStates", activeStateNames.toString());
        Logger.recordOutput("LEDs/Time", time);
        Logger.recordOutput("LEDs/RobotSpeedMultiplier", robotSpeedMultiplier);

        io.pushLEDs(colors);

        LoggedTracer.record("LEDs");
    }
}
