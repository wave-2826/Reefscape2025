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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
        boolean isHorizontal, double shiftLeft) {
        Rotation2d rot = alliance == DriverStation.Alliance.Red
            ? FieldMirroringUtils.flip(station.startingPose.getRotation())
            : station.startingPose.getRotation();
        Translation2d pos = alliance == DriverStation.Alliance.Red
            ? FieldMirroringUtils.flip(station.startingPose.getTranslation())
            : station.startingPose.getTranslation();

        pos = pos.plus(new Translation2d(Units.inchesToMeters(1), shiftLeft).rotateBy(rot));

        LinearVelocity xSpeed = MetersPerSecond.of(0.75 + Math.random() * 0.75);
        LinearVelocity ySpeed = MetersPerSecond.of(Math.random() * 0.75 - 0.325);
        AngularVelocity thetaSpeed = DegreesPerSecond.of(Math.random() * 360 - 180);
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed),
            rot);

        return isHorizontal
            ? new ReefscapeCoralOnFly(pos, new Translation2d(), speeds, rot.rotateBy(Rotation2d.kCCW_90deg),
                Centimeters.of(98), MetersPerSecond.of(0), Degrees.of(0))
            : new ReefscapeCoralOnFly(pos, new Translation2d(), speeds, rot, Centimeters.of(98), MetersPerSecond.of(0),
                Degrees.of(-50));
    }

    public static Command dropCoralCommand(boolean onRight, boolean shiftForward) {
        return Commands.runOnce(() -> {
            var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
            double shiftLeft = shiftForward ? Units.inchesToMeters(onRight ? -30 : 30) : 0;
            SimulatedArena.getInstance().addGamePieceProjectile(
                dropFromCoralStation(onRight ? CoralStationsSide.RIGHT_STATION : CoralStationsSide.LEFT_STATION,
                    alliance, false, shiftLeft));
        }).ignoringDisable(true);
    }

    public void configureControls() {
        buttonTrigger(1).onTrue(Commands.runOnce(IntakeIOSim::addCoral).ignoringDisable(true));

        buttonTrigger(2).onTrue(dropCoralCommand(false, false));
        buttonTrigger(3).onTrue(dropCoralCommand(true, false));
        buttonTrigger(5).onTrue(dropCoralCommand(false, true));
        buttonTrigger(6).onTrue(dropCoralCommand(true, true));

        SmartDashboard.putData("SimControls/Drop Left Station", dropCoralCommand(false, false));
        SmartDashboard.putData("SimControls/Drop Right Station", dropCoralCommand(true, false));
    }
}
