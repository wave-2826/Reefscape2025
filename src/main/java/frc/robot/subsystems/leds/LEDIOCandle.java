package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * LED IO that sends data to a CANdle. Note that we need to send one CAN message per block of same-colored LEDs to the
 * CANdle since there's no way to send a buffer of LED colors. This is extremely inefficient, so we don't run a CANdle
 * over CAN (despite its name) on our robot at the moment.
 */
public class LEDIOCandle implements LEDIO {
    private final CANdle candle;

    public LEDIOCandle() {
        candle = new CANdle(LEDConstants.candleId);

        CANdleConfiguration configSettings = new CANdleConfiguration();
        configSettings.statusLedOffWhenActive = true;
        configSettings.disableWhenLOS = false;
        configSettings.stripType = LEDStripType.GRB;
        configSettings.brightnessScalar = 1.0;
        configSettings.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(configSettings, 100);
    }

    @Override
    public void pushLEDs(Color[] colors) {
        int span = 1;
        for(int i = 0; i < LEDConstants.ledCount; i++) {
            Color8Bit color8Bit = convertTo8Bit(colors[i]);

            while(i + span < LEDConstants.ledCount && convertTo8Bit(colors[i + span]) == color8Bit) {
                span++;
            }

            candle.setLEDs(color8Bit.red, color8Bit.green, color8Bit.blue, 0, i - span + 1, span);

            i += span - 1;
            span = 1;
        }
    }

    private static Color8Bit convertTo8Bit(Color color) {
        return new Color8Bit((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
    }
}
