package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ArmVisualizer {
    private final String name;

    private final LoggedMechanism2d armMechanism;
    private final LoggedMechanismLigament2d elevatorHeightLigament;
    private final LoggedMechanismLigament2d armPitchLigament;

    public ArmVisualizer(String name) {
        this.name = name;

        // TODO: Derive from robot base and elevator maximum extension
        armMechanism = new LoggedMechanism2d(Units.inchesToMeters(30), Units.feetToMeters(7), new Color8Bit("#FF0000"));

        var armRoot = armMechanism.getRoot(name + "Root", elevatorOrigin.getX(), elevatorOrigin.getY());
        elevatorHeightLigament = armRoot.append(new LoggedMechanismLigament2d(name + "Elevator",
            ArmConstants.ElevatorConstants.resetSwitchHeight.in(Meters), Math.PI / 2., 10.0, new Color8Bit("#00FF00")));
        armPitchLigament = armRoot.append(new LoggedMechanismLigament2d(name + "ArmPitch",
            ArmConstants.PitchWristConstants.armLengthMeters, 0., 10.0, new Color8Bit("#0000FF")));
    }

    /**
     * Updates the arm visualization.
     * @param elevatorHeightMeters The height of the elevator relative to its hardstop.
     * @param armPitch
     * @param wristRotation
     * @param hasGamePiece
     */
    public void update(double elevatorHeightMeters, Rotation2d armPitch, Rotation2d wristRotation,
        boolean hasGamePiece) {
        elevatorHeightLigament.setLength(elevatorHeightMeters);
        armPitchLigament.setAngle(armPitch);
        Logger.recordOutput("Mechanism2d/" + name, armMechanism);

        // The second stage is technically ambiguous in position, but we assume that it's at the minimum height so that the
        // carriage is at the top.
        final double outerStageHeight = Math
            .max(elevatorHeightMeters - ArmConstants.ElevatorConstants.outerStageMaximumHeight.in(Meters), 0.0);
        final double carriageHeight = elevatorHeightMeters;

        Pose3d armPose = new Pose3d(elevatorOrigin.plus(elevatorToCarriage).plus(carriageToPivot),
            new Rotation3d(wristRotation.getRadians(), armPitch.getRadians(), 0.0));

        Logger.recordOutput("Mechanism3d/" + name + "/Arm",
            // Outer stage    
            new Pose3d(elevatorOrigin.plus(elevatorToOuterStage)
                .plus(new Translation3d(outerStageHeight, new Rotation3d(0.0, 0.0, 0.0))), new Rotation3d()),
            // Inner stage (carriage)
            new Pose3d(elevatorOrigin.plus(elevatorToCarriage)
                .plus(new Translation3d(carriageHeight, new Rotation3d(0.0, 0.0, 0.0))), new Rotation3d()),
            // Arm
            armPose);

        if(hasGamePiece) {
            Logger
                .recordOutput("Mechanism3d/" + name + "/Algae",
                    new Pose3d(RobotState.getInstance().getEstimatedPose())
                        .transformBy(new Transform3d(Pose3d.kZero, pivotPose3d))
                        .transformBy(new Transform3d(
                            ArmConstants.PitchWristConstants.armLengthMeters + Units.inchesToMeters(4.0), 0.0, -0.1,
                            Rotation3d.kZero))
                        .getTranslation());
        } else {
            Logger.recordOutput("Mechanism3d/" + name + "/Algae", new Translation3d[] {});
        }
    }
}
