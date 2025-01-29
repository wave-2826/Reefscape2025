package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;

public interface LEDIO {
    public default void pushLEDs() {
    }

    public default void setLEDColor(int index, Color color) {
    }
}
