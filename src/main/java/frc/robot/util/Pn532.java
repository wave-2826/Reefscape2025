package frc.robot.util;

import java.io.UnsupportedEncodingException;
import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

/**
 * Super sketchy and inflexible Pn532 implementation for our battery tracking. Credit to team 5924 for many of these
 * implementation details! See https://github.com/Team5924/SPI-RFID-Example/
 */
public class Pn532 implements AutoCloseable {
    private final SPI connection;

    // https://github.com/elechouse/PN532/blob/PN532_HSU/PN532_SPI/PN532_SPI.cpp
    // https://github.com/elechouse/PN532/blob/12663d9dc35e081277750f11fcbfc57e693b8768/PN532/PN532Interface.h#L6

    private static final byte PN532_PREAMBLE = (byte) 0x00;
    private static final byte PN532_STARTCODE1 = (byte) 0x00;
    private static final byte PN532_STARTCODE2 = (byte) 0xFF;
    private static final byte PN532_POSTAMBLE = (byte) 0x00;

    private static final byte PN532_HOSTTOPN532 = (byte) 0xD4;
    // private static final byte PN532_PN532TOHOST = (byte) 0xD5;

    private static final byte PN532_COMMAND_GETFIRMWAREVERSION = (byte) 0x02;

    private static final byte[] readCommand = new byte[] {
        (byte) 0x40, // Command: InDataExchange
        (byte) 0x01, // Target number
        (byte) 0x30, // Read type 2 NFC tags
        (byte) 0x00, // The block number to read; user data usually starts at block 4
    };

    private static final byte PN532_SPI_DATA_WRITE = (byte) 0x01;
    private static final byte PN532_SPI_STATUS_READ = (byte) 0x02;
    private static final byte PN532_SPI_DATA_READ = (byte) 0x03;
    private static final byte PN532_SPI_READY = 0x01;

    private static final byte[] PN532_ACKNOWLEDGEMENT = {
        (byte) 0x00, (byte) 0x00, (byte) 0xFF, (byte) 0x00, (byte) 0xFF, (byte) 0x00
    };
    private static final byte[] PN532_RESPONSE_FIRMWAREVERS = {
        (byte) 0x00, (byte) 0x00, (byte) 0xFF, (byte) 0x06, (byte) 0xFA, (byte) 0xD5
    };

    public Pn532() {
        connection = new SPI(SPI.Port.kOnboardCS0);
        connection.setClockRate(1_000_000); // The maximum rate the Pn532 supports is 5Mhz
        connection.setMode(SPI.Mode.kMode0); // The Pn532 only supports mode 0.
        // connection.setMode(SPI.Mode.kMode3); // Or not..??
        connection.setChipSelectActiveLow();

        try {
            if(!waitForReady(0.5)) {
                DriverStation.reportError("Error reading NFC: device wasn't ready when waiting for acknowledgement.",
                    false);
            } else {
                System.out.println("Connected to Pn532 with firmware version " + getFirmwareVersion());
            }
        } catch(InterruptedException e) {
            DriverStation.reportError("Error reading NFC: Interrupted while waiting for device to be ready.", false);
        }
    }

    private static byte reverseByte(byte b) {
        byte reversedByte = 0;
        for(int bit = 0; bit < 8; bit++) {
            reversedByte = (byte) ((reversedByte << 1) | (b & 1));
            b = (byte) (b >> 1);
        }
        return reversedByte;
    }

    private static byte[] reverseBytes(byte[] bytes) {
        byte[] reversedBytes = new byte[bytes.length];
        for(int i = 0; i < bytes.length; i++) {
            reversedBytes[i] = reverseByte(bytes[i]);
        }
        return reversedBytes;
    }

    private int writeCommand(byte[] command) {
        int checksum;
        int length = command.length;

        byte[] p = new byte[9 + length];
        length++;

        p[0] = PN532_SPI_DATA_WRITE;
        p[1] = PN532_PREAMBLE;
        p[2] = PN532_STARTCODE1;
        p[3] = PN532_STARTCODE2;
        checksum = PN532_PREAMBLE + PN532_STARTCODE1 + PN532_STARTCODE2;

        p[4] = (byte) length;
        p[5] = (byte) ((int) ((byte) ~length) + 1);
        p[6] = PN532_HOSTTOPN532;
        checksum += PN532_HOSTTOPN532;

        for(int i = 0; i < length - 1; i++) {
            p[i + 7] = command[i];

            checksum += command[i];
        }
        int index = 6 + length;
        p[index] = (byte) ~checksum;
        p[index + 1] = PN532_POSTAMBLE;

        return connection.write(reverseBytes(p), 8 + length);
    }

    /**
     * Determines if the device is ready.
     * @return True if the device is ready.
     * @throws InterruptedException
     */
    private boolean isReady() throws InterruptedException {
        byte[] statusReadCommand = {
            reverseByte(PN532_SPI_STATUS_READ)
        };

        // if(connection.write(statusReadCommand, statusReadCommand.length) == -1) {
        //     DriverStation.reportError("Error reading NFC: unable to write when checking ready status.", false);
        //     return false;
        // }

        // Thread.sleep(10);

        // if(connection.read(true, reply, reply.length) == -1) {
        //     DriverStation.reportError("Error reading NFC: unable to read when checking ready status.", false);
        //     return false;
        // }

        byte[] reply = new byte[1];
        if(connection.transaction(statusReadCommand, reply, 1) == -1) {
            DriverStation.reportError("Error reading NFC: unable to read when checking ready status.", false);
            return false;
        }

        // boolean replyCorrect = (reverseByte(reply[0]) & 1) == 1;
        // This is just for testing... the code will probably break.
        boolean replyCorrect = true;

        if(!replyCorrect) {
            DriverStation.reportError("Error reading NFC: Got response, but it was invalid (" + reverseByte(reply[0])
                + ", expected " + PN532_SPI_READY + ").", false);
            return false;
        }

        return true;
    }

    /**
     * Waits for the device to be ready.
     * @return True if the device was successfully readied.
     * @throws InterruptedException
     */
    private boolean waitForReady(double timeoutSeconds) throws InterruptedException {
        Timer timer = new Timer();
        timer.start();
        while(!isReady()) {
            if(timer.hasElapsed(timeoutSeconds)) {
                DriverStation.reportError("Error reading NFC: timed out while waiting for ready.", false);
                return false;
            }
            Thread.sleep(10);
        }
        return true;
    }

    /**
     * Reads an acknowledgement from the Pn532.
     * @return
     */
    private boolean readAcknowledgement() {
        byte[] command = {
            reverseByte(PN532_SPI_DATA_READ), 0x00, 0x00, 0x00, 0x00, 0x00
        };
        byte[] buffer = new byte[6];

        if(connection.transaction(command, buffer, 6) == -1) {
            DriverStation.reportError("Error reading NFC: reading acknowledgement failed.", false);
            return false;
        }

        return buffer.equals(PN532_ACKNOWLEDGEMENT);
    }

    /**
     * Sends a command and checks for an acknowledgement.
     * @return
     */
    private boolean sendCommandCheckAck(byte[] command, double timeoutSeconds) throws InterruptedException {
        writeCommand(command);

        if(!waitForReady(timeoutSeconds)) {
            DriverStation.reportError("Error reading NFC: device wasn't ready when waiting for acknowledgement.",
                false);
            return false;
        }
        if(!readAcknowledgement()) {
            DriverStation.reportError("Error reading NFC: reading acknowledgement failed.", false);
            return false;
        }

        // Wait for chip to say its ready!
        if(!waitForReady(timeoutSeconds)) {
            DriverStation.reportError("Error reading NFC: device wasn't ready when waiting after acknowledgement.",
                false);
            return false;
        }

        return true; // ack'd command
    }

    private boolean readData(byte[] buffer) {
        byte[] cmd = {
            reverseByte(PN532_SPI_DATA_READ)
        };
        if(connection.write(cmd, 1) == -1) {
            DriverStation.reportError("Error reading NFC: failed to write data read command.", false);
            return false;
        }
        if(connection.read(false, buffer, buffer.length) == -1) {
            DriverStation.reportError("Error reading NFC: failed to read data.", false);
            return false;
        }

        for(int i = 0; i < buffer.length; i++) {
            buffer[i] = reverseByte(buffer[i]);
        }

        return true;
    }

    /**
     * Gets the board's firmware version.
     * @return
     */
    private int getFirmwareVersion() throws InterruptedException {
        int response;

        byte[] pn532_packetbuffer = {
            PN532_COMMAND_GETFIRMWAREVERSION
        };

        if(!sendCommandCheckAck(pn532_packetbuffer, 0.5)) { return 0; }

        byte[] buffer = new byte[PN532_RESPONSE_FIRMWAREVERS.length];
        readData(buffer);

        // check some basic stuff
        if(!buffer.equals(PN532_RESPONSE_FIRMWAREVERS)) { return 0; }

        int offset = 7;
        response = buffer[offset++];
        response <<= 8;
        response |= buffer[offset++];
        response <<= 8;
        response |= buffer[offset++];
        response <<= 8;
        response |= buffer[offset++];

        return response;
    }

    /**
     * Sends the read command to the PN532.
     * @param blockNumber The block number to read. User data usually starts at block 4.
     * @return True if the command was sent successfully.
     * @throws InterruptedException
     */
    private boolean sendReadCommand(int blockNumber) throws InterruptedException {
        byte[] command = Arrays.copyOf(readCommand, readCommand.length);
        command[3] = (byte) blockNumber; // Set the block number to read

        // Send the command
        return sendCommandCheckAck(command, 0.5);
    }

    private byte[] readTagData() {
        // First, we need to read the initial response frame to determine the length.
        byte[] header = new byte[6];
        if(!readData(header)) {
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
        if(!readData(response)) {
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

        return readTagData();
    }

    public String readAsciiBytes() {
        try {
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
