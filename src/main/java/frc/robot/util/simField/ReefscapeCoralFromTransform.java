package frc.robot.util.simField;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class ReefscapeCoralFromTransform extends GamePieceProjectile {
    public ReefscapeCoralFromTransform(Translation2d robotPosition, Transform3d coralTransformOnRobot,
        ChassisSpeeds chassisSpeeds, Rotation2d shooterFacing, Distance initialHeight, LinearVelocity launchingSpeed,
        Angle shooterAngle) {
        super(ReefscapeCoralOnField.REEFSCAPE_CORAL_INFO,
            robotPosition.plus(coralTransformOnRobot.getTranslation().toTranslation2d().rotateBy(shooterFacing)),
            calculateInitialProjectileVelocityMPS(coralTransformOnRobot.getTranslation().toTranslation2d(),
                chassisSpeeds, shooterFacing, launchingSpeed.in(MetersPerSecond) * Math.cos(shooterAngle.in(Radians))),
            initialHeight.in(Meters), launchingSpeed.in(MetersPerSecond) * Math.sin(shooterAngle.in(Radians)),
            coralTransformOnRobot.getRotation().rotateBy(new Rotation3d(0, 0, shooterFacing.getRadians())));

        super.enableBecomesGamePieceOnFieldAfterTouchGround();
        super.withTouchGroundHeight(0.2);
    }

    private static Translation2d calculateInitialProjectileVelocityMPS(Translation2d coralPosition,
        ChassisSpeeds chassisSpeeds, Rotation2d chassisFacing, double groundSpeedMPS) {
        final Translation2d chassisTranslationalVelocity = new Translation2d(chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond),
            shooterGroundVelocityDueToChassisRotation = coralPosition.rotateBy(chassisFacing)
                .rotateBy(Rotation2d.fromDegrees(90)).times(chassisSpeeds.omegaRadiansPerSecond),
            shooterGroundVelocity = chassisTranslationalVelocity.plus(shooterGroundVelocityDueToChassisRotation);

        return shooterGroundVelocity.plus(new Translation2d(groundSpeedMPS, chassisFacing));
    }
}
