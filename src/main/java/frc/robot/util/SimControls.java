package frc.robot.util;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.ironmaple.utils.FieldMirroringUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.intake.IntakeIOSim;

public class SimControls {
    private static final SimControls instance = new SimControls();

    private Joystick simJoystick = new Joystick(2);

    public static SimControls getInstance() {
        return instance;
    }

    private SimControls() {
        // This is a singleton
    }

    private Trigger buttonTrigger(int button) {
        var eventLoop = CommandScheduler.getInstance().getDefaultButtonLoop();
        return new Trigger(simJoystick.button(button, eventLoop));
    }

    private enum CoralStationsSide {
        LEFT_STATION(new Pose2d(0.89, 7.32, Rotation2d.fromDegrees(-54))), RIGHT_STATION(
            new Pose2d(0.89, 0.6, Rotation2d.fromDegrees(54)));

        private final Pose2d startingPose;

        CoralStationsSide(Pose2d startingPose) {
            this.startingPose = startingPose;
        }
    }

    private static ReefscapeCoralOnFly dropFromCoralStation(CoralStationsSide station, DriverStation.Alliance alliance,
        boolean isHorizontal) {
        Rotation2d rot = alliance == DriverStation.Alliance.Red
            ? FieldMirroringUtils.flip(station.startingPose.getRotation())
            : station.startingPose.getRotation();
        Translation2d pos = alliance == DriverStation.Alliance.Red
            ? FieldMirroringUtils.flip(station.startingPose.getTranslation())
            : station.startingPose.getTranslation();

        LinearVelocity xSpeed = MetersPerSecond.of(1.5 + Math.random() * 2.5 - 1.25);
        LinearVelocity ySpeed = MetersPerSecond.of(Math.random() * 1.5 - 0.75);
        AngularVelocity thetaSpeed = DegreesPerSecond.of(Math.random() * 360 - 180);

        return isHorizontal
            ? new ReefscapeCoralOnFly(pos, new Translation2d(),
                ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed), rot),
                rot.rotateBy(Rotation2d.kCCW_90deg), Centimeters.of(98), MetersPerSecond.of(0), Degrees.of(0))
            : new ReefscapeCoralOnFly(
                alliance == DriverStation.Alliance.Red ? FieldMirroringUtils.flip(station.startingPose.getTranslation())
                    : station.startingPose.getTranslation(),
                new Translation2d(), new ChassisSpeeds(),
                alliance == DriverStation.Alliance.Red ? FieldMirroringUtils.flip(station.startingPose.getRotation())
                    : station.startingPose.getRotation(),
                Centimeters.of(98), MetersPerSecond.of(3), Degrees.of(-50));
    }

    public void configureControls() {
        buttonTrigger(1).onTrue(Commands.runOnce(IntakeIOSim::addCoral));

        buttonTrigger(2).onTrue(Commands.runOnce(() -> {
            var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
            SimulatedArena.getInstance()
                .addGamePieceProjectile(dropFromCoralStation(CoralStationsSide.LEFT_STATION, alliance, true));
        }));
        buttonTrigger(3).onTrue(Commands.runOnce(() -> {
            var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
            SimulatedArena.getInstance()
                .addGamePieceProjectile(dropFromCoralStation(CoralStationsSide.RIGHT_STATION, alliance, true));
        }));
    }
}
