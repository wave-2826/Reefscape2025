package frc.robot.util;

import java.util.Arrays;

import edu.wpi.first.wpilibj.I2C;

/**
 * Super sketchy and inflexible Pn532 implementation for our battery tracking.
 */
public class Pn532 implements AutoCloseable {
    private final I2C connection;

    // https://github.com/elechouse/PN532/blob/PN532_HSU/PN532_I2C/PN532_I2C.cpp
    // https://github.com/elechouse/PN532/blob/12663d9dc35e081277750f11fcbfc57e693b8768/PN532/PN532Interface.h#L6
    byte[] readCommand = new byte[] {
        // TODO: Apparently we have to do some wakeup sequence? Idk...
        (byte) 0x00, // PN532_PREAMBLE
        (byte) 0x00, // PN532_STARTCODE1
        (byte) 0xFF, // PN532_STARTCODE2
        // uint8_t length = hlen + blen + 1;   // length of data field: TFI + DATA
        (byte) 0xD4, // 
        (byte) 0x40, (byte) 0x01, (byte) 0x30, (byte) 0x04
    };

    public Pn532() {
        // Note: do not use the onboard port on the roboRIO! It can cause system lockups.
        connection = new I2C(I2C.Port.kMXP, 0x24);
    }

    public void read4Bytes() {
        // boolean aborted = connection.writeBulk(readCommand);
        // if(aborted) {
        //     System.out.println("Error reading NFC tag: writing aborted!");
        //     return;
        // }

        // byte[] response = new byte[10];
        // aborted = connection.readOnly(response, response.length);
        // if(aborted) {
        //     System.out.println("Error reading NFC tag: read aborted!");
        //     return;
        // }

        // if(response[1] == (byte) 0x41 && response[2] == 0x00) {
        //     byte[] nfcData = Arrays.copyOfRange(response, 3, 7);
        //     System.out.println("NFC Data: " + Arrays.toString(nfcData));
        // } else {
        //     System.out.println("Error reading NFC tag: response doesn't match! " + Arrays.toString(response));
        // }
    }

    // private int[] readPassiveTargetID(int cardBaudRate, int timeout) {
    // }

    @Override
    public void close() {
        connection.close();
    }
}
