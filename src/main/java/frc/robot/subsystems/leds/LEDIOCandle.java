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
    private final Color8Bit[] ledBuffer;

    public LEDIOCandle() {
        candle = new CANdle(LEDConstants.candleId);

        CANdleConfiguration configSettings = new CANdleConfiguration();
        configSettings.statusLedOffWhenActive = true;
        configSettings.disableWhenLOS = false;
        configSettings.stripType = LEDStripType.GRB;
        configSettings.brightnessScalar = 1.0;
        configSettings.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(configSettings, 100);

        ledBuffer = new Color8Bit[LEDConstants.ledCount];
        for(int i = 0; i < LEDConstants.ledCount; i++) {
            ledBuffer[i] = convertTo8Bit(Color.kBlack);
        }
    }

    public void pushLEDs() {
        for(int i = 0; i < LEDConstants.ledCount; i++) {
            int count = 1;
            while(i + count < LEDConstants.ledCount && ledBuffer[i] == ledBuffer[i + count]) {
                count++;
            }
            candle.setLEDs(ledBuffer[i].red, ledBuffer[i].green, ledBuffer[i].blue, 0, i, count);
        }
    }

    public void setLEDColor(int index, Color color) {
        ledBuffer[index] = convertTo8Bit(color);
    }

    private static Color8Bit convertTo8Bit(Color color) {
        return new Color8Bit((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
    }
}
