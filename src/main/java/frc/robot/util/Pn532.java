package frc.robot.util;

import java.util.Arrays;

import edu.wpi.first.wpilibj.I2C;

/**
 * Super sketchy and inflexible Pn532 implementation for our battery tracking.
 */
public class Pn532 implements AutoCloseable {
    private final I2C connection;

    byte[] readCommand = new byte[] {
        // TODO: Apparently we have to do some wakeup sequence? Idk...
        (byte) 0xD4, (byte) 0x40, (byte) 0x01, (byte) 0x30, (byte) 0x04
    };

    public Pn532() {
        // Note: do not use the onboard port! It can cause system lockups.
        connection = new I2C(I2C.Port.kMXP, 0x24);
    }

    public void read4Bytes() {
        connection.writeBulk(readCommand);

        byte[] response = new byte[10];
        connection.readOnly(response, response.length);

        if(response[1] == (byte) 0x41 && response[2] == 0x00) {
            byte[] nfcData = Arrays.copyOfRange(response, 3, 7);
            System.out.println("NFC Data: " + Arrays.toString(nfcData));
        } else {
            System.out.println("Error reading NFC tag");
        }
    }

    @Override
    public void close() {
        connection.close();
    }
}
