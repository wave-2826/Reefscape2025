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
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.SimRobotGamePiece;

public class ArmVisualizer {
    private final String name;

    private final LoggedMechanism2d armMechanism;
    private final LoggedMechanismLigament2d elevatorHeightLigament;
    private final LoggedMechanismLigament2d armPitchLigament;

    public ArmVisualizer(String name) {
        this.name = name;

        armMechanism = new LoggedMechanism2d(Units.inchesToMeters(60), Units.feetToMeters(7), new Color8Bit("#050515"));

        var origin = ArmConstants.ElevatorConstants.elevatorOrigin;

        var armRoot = armMechanism.getRoot(name + "Root", origin.getX(), origin.getY());
        elevatorHeightLigament = armRoot
            .append(new LoggedMechanismLigament2d(name + "Elevator", 0.0, 90., 10.0, new Color8Bit("#00FF00")));
        armPitchLigament = elevatorHeightLigament.append(new LoggedMechanismLigament2d(name + "ArmPitch",
            ArmConstants.ShoulderConstants.armLength.in(Meters), 0., 10.0, new Color8Bit("#0000FF")));
    }

    /** A constant offset to correct for the simplified model not exactly matching the real robot model. */
    private static double carriageOffsetMeters = Units.inchesToMeters(12.);

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
        final double carriageHeight = elevatorHeightMeters;
        final double passiveStageHeight = Math
            .max(carriageHeight + ArmConstants.ElevatorConstants.carriageHeight.in(Meters)
                - ArmConstants.ElevatorConstants.carriageMaxHeight.in(Meters) - carriageOffsetMeters, 0.0);

        var elevatorOrigin = ArmConstants.ElevatorConstants.elevatorOrigin;

        Pose3d carriagePose = new Pose3d(elevatorOrigin.plus(ArmConstants.ElevatorConstants.elevatorToCarriage)
            .plus(new Translation3d(0.0, 0.0, carriageHeight)), new Rotation3d());
        Pose3d armShoulderPose = carriagePose.plus(new Transform3d(ArmConstants.ShoulderConstants.carriageToShoulder,
            new Rotation3d(0.0, Math.PI / 2 - armPitch.getRadians(), 0.0)));
        Pose3d armWristPose = armShoulderPose.plus(new Transform3d(ArmConstants.ShoulderConstants.shoulderToWrist,
            new Rotation3d(0.0, 0.0, wristRotation.getRadians())));

        Logger.recordOutput("Mechanism3d/" + name + "/Arm",
            // Outer stage    
            new Pose3d(elevatorOrigin.plus(ArmConstants.ElevatorConstants.elevatorToPassiveStage)
                .plus(new Translation3d(0.0, 0.0, passiveStageHeight)), new Rotation3d()),
            // Inner stage (carriage)
            carriagePose,
            // Arm
            armShoulderPose, armWristPose);

        if(hasGamePiece) {
            Pose3d coralPose = armWristPose.plus(ArmConstants.ShoulderConstants.wristToCoral);
            Transform3d coralTransform = new Transform3d(coralPose.getTranslation(), coralPose.getRotation());
            SimRobotGamePiece.setCoralTransform(coralTransform);
        }
    }
}
