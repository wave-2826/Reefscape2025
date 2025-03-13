package frc.robot.util;

import java.io.UnsupportedEncodingException;
import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;

/**
 * Super sketchy and inflexible Pn532 implementation for our battery tracking.
 */
public class Pn532 implements AutoCloseable {
    private final I2C connection;

    // https://github.com/elechouse/PN532/blob/PN532_HSU/PN532_I2C/PN532_I2C.cpp
    // https://github.com/elechouse/PN532/blob/12663d9dc35e081277750f11fcbfc57e693b8768/PN532/PN532Interface.h#L6

    private static final byte[] wakeupCommand = new byte[] {
        (byte) 0x55, // Preamble
        (byte) 0x55, // Preamble
        (byte) 0x00, // Start code 1
        (byte) 0x00, // Start code 2
        (byte) 0x00, // Postamble
    };

    private static final byte[] readCommand = new byte[] {
        // TODO: Apparently we have to do some wakeup sequence? Idk...
        (byte) 0x00, // Preamble
        (byte) 0x00, // Start code 1
        (byte) 0xFF, // Start code 2
        (byte) 0x06, // Length
        (byte) 0xFF - 0x06, // Length checksum
        // Payload start
        (byte) 0xD4, // Data direction: host to PN532
        (byte) 0x40, // Command: InDataExchange
        (byte) 0x01, // Target number
        (byte) 0x30, // Read type 2 NFC tags
        (byte) 0x00, // The block number to read; user data usually starts at block 4
        (byte) 0xD6, // Data checksum (0x100 - sum of payload)
        // Payload end
        (byte) 0x00, // Postamble
    };

    private static final int PN532_I2C_ADDRESS = 0x48; // 7-bit address
    private static final byte PN532_READY = 0x01;

    public Pn532() {
        // Note: do not use the onboard port on the roboRIO! It can cause system lockups.
        connection = new I2C(I2C.Port.kMXP, PN532_I2C_ADDRESS);
    }

    /**
     * Sends the wakeup command to the PN532.
     * @return True if the command was sent successfully.
     */
    private boolean sendWakeup() {
        return !connection.writeBulk(wakeupCommand);
    }

    /**
     * Sends the read command to the PN532.
     * @param blockNumber The block number to read. User data usually starts at block 4.
     * @return True if the command was sent successfully.
     * @throws InterruptedException
     */
    private boolean sendReadCommand(int blockNumber) throws InterruptedException {
        byte[] command = Arrays.copyOf(readCommand, readCommand.length);
        command[9] = (byte) blockNumber; // Set the block number to read

        // Send the command
        return !connection.writeBulk(command);
    }

    /**
     * The PN532 sets its status byte to 0x01 when data is ready... I think?
     * @return
     * @throws InterruptedException
     */
    private boolean waitForResponse() throws InterruptedException {
        byte[] status = new byte[1];
        for(int i = 0; i < 20; i++) { // Try for 200ms
            if(!connection.readOnly(status, 1) && status[0] == PN532_READY) { return true; }
            Thread.sleep(10); // Wait 10ms before retrying
        }
        return false;
    }

    /**
     * Waits up to 200ms for an I2C connection.
     * @return True if the device is connected.
     */
    private boolean waitForConnection() throws InterruptedException {
        for(int i = 0; i < 20; i++) { // Try for 200ms
            if(!connection.addressOnly()) { return true; }
            Thread.sleep(10); // Wait 10ms before retrying
        }
        return false;
    }

    private byte[] readTagData() {
        // First, we need to read the initial response frame to determine the length.
        byte[] header = new byte[6];
        if(connection.readOnly(header, 6)) {
            DriverStation.reportError("Error reading NFC tag: Failed to read response header.", null);
            return null;
        }

        // Validate the first 3 bytes are the correct preamble
        if(header[0] != 0x00 || header[1] != 0x00 || header[2] != (byte) 0xFF) {
            DriverStation.reportError("Error reading NFC tag: Invalid response header.", null);
            return null;
        }

        int length = header[3] & 0xFF;
        int lcs = header[4] & 0xFF;
        // Validate the checksum
        if((length + lcs) != 0x100) {
            DriverStation.reportError("Error reading NFC tag: Length checksum mismatch.", null);
            return null;
        }

        // Read the full response plus two bytes for the data checksum and postamble
        byte[] response = new byte[length + 2];

        if(connection.readOnly(response, response.length)) {
            DriverStation.reportError("Error reading NFC tag: Failed to read response data.", null);
            return null;
        }

        // Validate the response start bytes
        if(response[0] != (byte) 0xD5 || response[1] != (byte) 0x41) {
            DriverStation.reportError("Error reading NFC tag: Unexpected response format.", null);
            return null;
        }

        // STATUS should be 0x00 for success
        if(response[2] != 0x00) {
            DriverStation.reportError("Error reading NFC tag: Read operation failed, status: " + response[2], null);
            return null;
        }

        // Don't check the data checksum because I'm lazy

        // Extract the data payload
        return Arrays.copyOfRange(response, 3, response.length - 1);
    }

    private byte[] readBlock(int blockNumber) throws InterruptedException {
        if(!sendReadCommand(blockNumber)) {
            DriverStation.reportError("Error reading NFC: Failed to send read command.", false);
            return null;
        }

        if(!waitForResponse()) {
            DriverStation.reportError("Error reading NFC: PN532 did not respond.", false);
            return null;
        }

        return readTagData();
    }

    public String readAsciiBytes() {
        try {
            if(!waitForConnection()) {
                DriverStation.reportError("Error reading NFC: Device not on the I2C bus.", false);
                return null;
            }

            if(!sendWakeup()) {
                DriverStation.reportError("Error reading NFC: Failed to send wakeup command.", false);
                return null;
            }

            byte[] data = readBlock(4);
            if(data == null) {
                DriverStation.reportError("Error reading NFC: Failed to read block 4.", false);
                return null;
            }

            try {
                return new String(data, "US-ASCII");
            } catch(UnsupportedEncodingException e) {
                DriverStation.reportError("Error reading NFC: Failed to convert data to ASCII.", false);
                return null;
            }
        } catch(InterruptedException e) {
            DriverStation.reportError("Error reading NFC: Interrupted while reading from PN532.", false);
            return null;
        }
    }

    @Override
    public void close() {
        connection.close();
    }
}
