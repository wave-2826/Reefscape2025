package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ClimberVisualizer {
    private final String name;

    private final LoggedMechanism2d climberMechanism;
    private final LoggedMechanismLigament2d climberLigament;

    public ClimberVisualizer(String name) {
        this.name = name;

        climberMechanism = new LoggedMechanism2d(Units.inchesToMeters(30), Units.feetToMeters(7),
            new Color8Bit("#050515"));

        var origin = ClimberConstants.climberOrigin;
        var armRoot = climberMechanism.getRoot(name + "Root", origin.getX(), origin.getY());
        climberLigament = armRoot.append(new LoggedMechanismLigament2d(name,
            ClimberConstants.climberArmLength.in(Meters), 45., 10.0, new Color8Bit("#00FF00")));
    }

    /**
     * Updates the climber visualization.
     * @param rotation The rotation of the climber.
     */
    public void update(Rotation2d rotation) {
        climberLigament.setAngle(rotation.getDegrees());
        Logger.recordOutput("Mechanism2d/" + name, climberMechanism);

        var climberOrigin = ClimberConstants.climberOrigin;
        Logger.recordOutput("Mechanism3d/" + name + "/Climber",
            new Pose3d(climberOrigin, new Rotation3d(0.0, rotation.getDegrees(), 0.0)));
    }
}
