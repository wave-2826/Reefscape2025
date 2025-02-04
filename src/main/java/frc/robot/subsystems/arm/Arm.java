package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The arm subsystem. Manages the elevator, arm pitch, arm wrist rotation, and end effector. We combine all these into
 * one subsystem because they need to closely interact; most of the time, the elevator, arm pitch, and arm wrist move
 * simultaneously, and the end effector must work with the arm wrist while rotating because we use a coaxial drive
 * system for it.
 */
public class Arm extends SubsystemBase {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs;

    private ArmState targetState;

    public Arm(ArmIO io) {
        this.io = io;
        this.inputs = new ArmIOInputsAutoLogged();
    }

    @Override
    public void periodic() {

    }
}
