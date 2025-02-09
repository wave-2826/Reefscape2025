package frc.robot.subsystems.leds;

/**
 * Constants related to the LED subsystem.
 */
public class LEDConstants {
    /** The DIO port used for LEDs. */
    public static int ledDIOPort = 0;

    /** The CAN bus ID used for the CANdle, if it's being used over CAN. */
    public static int candleId = 53;

    /** If the LEDs have a GRB color order. */
    public static boolean isGRB = true;

    /** The number of LEDs on the robot. */
    public static int ledCount = 8;
}
