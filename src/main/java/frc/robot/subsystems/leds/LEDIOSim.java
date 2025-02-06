package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.util.Color;

/**
 * LED IO that sends LED data as hex strings on a string[] topic on NetworkTables. Used to display the LEDs on Elastic
 * dashboard in simulation. See
 * https://frc-elastic.gitbook.io/docs/additional-features-and-references/widgets-list-and-properties-reference#multi-color-view
 */
public class LEDIOSim implements LEDIO {
    private final String[] LEDData;

    public LEDIOSim() {
        LEDData = new String[LEDConstants.ledCount];

        for(int i = 0; i < LEDConstants.ledCount; i++) {
            LEDData[i] = "#000000";
        }
    }

    @Override
    public void pushLEDs(Color[] colors) {
        for(int i = 0; i < LEDConstants.ledCount; i++) {
            LEDData[i] = colors[i].toHexString();
        }

        Logger.recordOutput("LEDs/SimLEDs", LEDData);
    }
}
