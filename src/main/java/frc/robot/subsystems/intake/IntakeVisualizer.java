package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class IntakeVisualizer {
    private final String name;

    private final LoggedMechanism2d intakeMechanism;
    private final LoggedMechanismLigament2d intakeLigament;

    public IntakeVisualizer(String name) {
        this.name = name;

        intakeMechanism = new LoggedMechanism2d(Units.inchesToMeters(30), Units.feetToMeters(7),
            new Color8Bit("#051505"));

        var origin = IntakeConstants.intakeOrigin;
        var armRoot = intakeMechanism.getRoot(name + "Root", origin.getX(), origin.getY());
        intakeLigament = armRoot.append(new LoggedMechanismLigament2d(name, IntakeConstants.intakeLength.in(Meters),
            45., 10.0, new Color8Bit("#00FF00")));
    }

    /**
     * Updates the intake visualization.
     * @param pitch The pitch of the intake. 0 degrees is straight outward.
     */
    public void update(Rotation2d pitch) {
        intakeLigament.setAngle(pitch.getDegrees());
        Logger.recordOutput("Mechanism2d/" + name, intakeMechanism);

        var origin = IntakeConstants.intakeOrigin;
        Logger.recordOutput("Mechanism3d/" + name + "/Intake",
            new Pose3d(origin, new Rotation3d(0.0, pitch.getRadians(), 0.0)));
    }
}
