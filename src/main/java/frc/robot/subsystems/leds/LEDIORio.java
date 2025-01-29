package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/**
 * LED IO that sends pulse-train WS2812B data out of a RoboRIO DIO port. Note that this also applies to a CANdle
 * connected using pulse-train mode, which is how we currently connect our LEDs. Otherwise, we would need to send one
 * CAN message per block of same-colored LEDs to the CANdle, which is extremely inefficient.
 */
public class LEDIORio implements LEDIO {
    private final AddressableLED leds;
    private final AddressableLEDBuffer buffer;

    public LEDIORio() {
        leds = new AddressableLED(LEDConstants.ledDIOPort);
        buffer = new AddressableLEDBuffer(LEDConstants.ledCount);

        leds.setLength(LEDConstants.ledCount);
        leds.setData(buffer);
        leds.start();
    }

    public void pushLEDs() {
        leds.setData(buffer);
    }

    public void setLEDColor(int index, Color color) {
        if(LEDConstants.isGRB) color = convertToGRB(color);

        buffer.setLED(index, color);
    }

    static Color convertToGRB(Color color) {
        return new Color(color.green, color.red, color.blue);
    }
}
