package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The LED subsystem. Manages the LED states.
 */
public class LEDs extends SubsystemBase {
    LEDIO io;

    public LEDs(LEDIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        // TODO: LED statess
        io.pushLEDs();
    }
}
