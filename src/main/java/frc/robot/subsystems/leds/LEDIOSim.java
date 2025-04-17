package frc.robot.subsystems.leds;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.wpilibj.util.Color;

/**
 * LED IO that sends LED data as hex strings on a string[] topic on NetworkTables. Used to display the LEDs on Elastic
 * dashboard in simulation. See
 * https://frc-elastic.gitbook.io/docs/additional-features-and-references/widgets-list-and-properties-reference#multi-color-view
 */
public class LEDIOSim implements LEDIO {
    private final String[] LEDData;
    private final StringArrayPublisher ledsPath = NetworkTableInstance.getDefault().getStringArrayTopic("Sim/LEDs")
        .publish();

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

        ledsPath.accept(LEDData);
    }
}
