package frc.robot.commands.auto;

import java.util.Comparator;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.RobotState;
import frc.robot.commands.drive.DriveToPose;

public class TrackCoral extends DriveToPose {
    private static final LoggedTunableNumber lookAheadSecs = new LoggedTunableNumber("TrackCoral/LookAheadSecs", 0.3);
    private static final LoggedTunableNumber angleDifferenceWeight = new LoggedTunableNumber(
        "TrackCoral/AngleDifferenceWeight", 0.3);
    private static final LoggedTunableNumber coralMaxDistance = new LoggedTunableNumber("TrackCoral/CoralMaxDistance",
        1.75);
    private static final LoggedTunableNumber coralMaxAngleDeg = new LoggedTunableNumber(
        "TrackCoral/CoralMaxAngleDegrees", 70.0);

    private final Debouncer notFoundTimeout = new Debouncer(0.5, Debouncer.DebounceType.kRising);
    private final Runnable grabbingFailed;

    public TrackCoral(Drive drive, Runnable grabbingFailed) {
        super(drive, () -> {
            RobotState robotState = RobotState.getInstance();
            Pose2d robot = robotState.getPose();

            Pose2d predictedRobot = robotState.getLookaheadPose(lookAheadSecs.get());
            Logger.recordOutput("TrackCoral/LookAheadPose", predictedRobot);

            Optional<Translation2d> trackedCoralPosition = robotState.getCoralTranslations()
                .min(Comparator.comparingDouble(coral -> coral.getDistance(predictedRobot.getTranslation())
                    + Math.abs(coral.minus(robot.getTranslation()).getAngle().minus(robot.getRotation()).getRadians()
                        * angleDifferenceWeight.get())))
                .filter(coral -> coral.getDistance(predictedRobot.getTranslation()) <= coralMaxDistance.get()
                    && Math.abs(predictedRobot.getRotation().rotateBy(Rotation2d.kPi).getDegrees()
                        - (coral.minus(predictedRobot.getTranslation()).getAngle().getDegrees())) <= coralMaxAngleDeg
                            .get());

            Pose2d target = trackedCoralPosition.<Pose2d>map(coralPosition -> {
                Logger.recordOutput("TrackCoral/TargetedCoral", new Translation2d[] {
                    coralPosition
                });
                return new Pose2d(coralPosition, robot.getTranslation().minus(coralPosition).getAngle())
                    .transformBy(new Transform2d(DriveConstants.bumperSizeMeters / 2.0, 0.0, Rotation2d.kZero));
            }).orElseGet(() -> {
                Logger.recordOutput("TrackCoral/TargetedCoral", new Translation2d[] {});
                return RobotState.getInstance().getPose();
            });

            return target;
        });

        this.grabbingFailed = grabbingFailed;
    }

    @Override
    public void initialize() {
        super.initialize();
        notFoundTimeout.calculate(false);
    }

    @Override
    public boolean isFinished() {
        boolean timedOut = notFoundTimeout.calculate(RobotState.getInstance().getCoralTranslations().count() == 0);
        if(timedOut && grabbingFailed != null) grabbingFailed.run();
        return timedOut;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        Logger.recordOutput("TrackCoral/TargetedCoral", new Translation2d[] {});
    }
}
