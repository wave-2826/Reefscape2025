package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.function.BooleanConsumer;
import frc.robot.RobotState;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.LEDState;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ReefTarget;

public class ReefLineupCommand extends DriveToPose {
    private static final LoggedTunableNumber[] lineupDistances = new LoggedTunableNumber[] {
        new LoggedTunableNumber("AutoScore/L1ReefLineupDistance", 33.5),
        new LoggedTunableNumber("AutoScore/L2ReefLineupDistance", 20.5),
        new LoggedTunableNumber("AutoScore/L3ReefLineupDistance", 21.75),
        new LoggedTunableNumber("AutoScore/L4ReefLineupDistance", 24)
    };

    private static final LoggedTunableNumber centerDistanceTweak = new LoggedTunableNumber(//
        "AutoScore/CenterDistanceTweak", 0.5);
    private static final LoggedTunableNumber centerDistanceTweakRight = new LoggedTunableNumber(//
        "AutoScore/CenterPositionTweakRight", -0.25);
    private static final LoggedTunableNumber safeOffsetOutward = new LoggedTunableNumber(//
        "AutoScore/SafeOffsetOutward", 14.);

    private final LEDs leds;

    private final Optional<BooleanSupplier> finishSequence;
    private final BooleanConsumer lineupFeedback;

    private final Debouncer ledDebouncer = new Debouncer(0.3, DebounceType.kFalling);
    private final Debouncer feedbackDebouncer = new Debouncer(0.15, DebounceType.kFalling);

    private static Transform2d flipFieldTransform(Transform2d transform) {
        return new Transform2d(new Translation2d(-transform.getX(), -transform.getY()),
            transform.getRotation().unaryMinus());
    }

    /**
     * Gets the pose to line up to for the given target. This is the pose that the robot should be at when it is fully
     * aligned to the target.
     */
    public static Pose2d getLineupPose(ReefTarget target) {
        if(RobotState.getInstance().isReefLineupSafe() && !IntakeCommands.waitingForPiece) {
            return getLineupPose(target, 0.);
        } else {
            return getSafeLineupPose(target);
        }
    }

    private static Pose2d getLineupPose(ReefTarget target, double offsetOutwardInches) {
        double distanceAwayInches;
        if(target.isAlgaePosition()) {
            distanceAwayInches = 30;
        } else {
            distanceAwayInches = lineupDistances[target.level().ordinal()].get();
        }
        distanceAwayInches += offsetOutwardInches;

        boolean alignCenter = target.isAlgaePosition();

        double centerDistance = FieldConstants.reefBranchSeparation.in(Meters) / 2.
            + Units.inchesToMeters(centerDistanceTweak.get());
        boolean isLeft = target.branch().isLeft;
        double horizontalOffset = (alignCenter ? 0. : (isLeft ? -centerDistance : centerDistance));

        Transform2d tagRelativeOffset = new Transform2d(new Translation2d(Units.inchesToMeters(distanceAwayInches),
            horizontalOffset + Units.inchesToMeters(centerDistanceTweakRight.get())), Rotation2d.k180deg);
        Pose2d fieldPose = target.branch().face.getTagPose().transformBy(tagRelativeOffset);

        return fieldPose;
    }

    /**
     * Gets a safe lineup pose based on a reef target, meaning one we can initially line up with before the elevator is
     * fully up.
     */
    private static Pose2d getSafeLineupPose(ReefTarget target) {
        return getLineupPose(target, safeOffsetOutward.get());
    }

    /**
     * A command that lines up the robot based on tracking the relative position of a single tag from the vision system
     * with PID control. We found this to be the most reliable mechanism to automatically align because it doesn't
     * depend on all tags the robot sees being in precise locations.
     * <p>
     * Internally, this actually _does_ track a field-relative position to allow us to take advantage of drivetrain
     * odometry to run the PID loop faster. However, the target pose is always relative to the tag being tracked.
     * <p>
     * If the tag being tracked is not seen, this falls back to full field-relative tracking.
     * @param drive
     * @param leds
     * @param face The face of the reef to line up with. This is used to determine which tag to track.
     * @param tagRelativeOffset The offset relative to the tag being tracked.
     * @param fieldRelativeOffset The offset relative to the field. This is flipped to be alliance-relative.
     * @param finishSequence If present, the reef lineup waits for this to be true before finishing. If not present, the
     *            reef lineup finishes when the target is fully aligned.
     * @param lineupFeedback A callback that is called every loop which can be used to provide feedback to the driver in
     *            teleop for when the robot is properly aligned. Can be null.
     */
    public ReefLineupCommand(Drive drive, LEDs leds, ReefTarget target, Supplier<Transform2d> fieldRelativeOffset,
        Optional<BooleanSupplier> finishSequence, BooleanConsumer lineupFeedback) {
        super(drive, () -> {
            Transform2d absoluteOffset = fieldRelativeOffset.get();
            if(AutoBuilder.shouldFlip()) {
                absoluteOffset = flipFieldTransform(absoluteOffset);
            }

            Pose2d pose = getLineupPose(target);
            return new Pose2d(pose.getTranslation().plus(absoluteOffset.getTranslation()),
                pose.getRotation().plus(absoluteOffset.getRotation()));
        }, () -> {
            return RobotState.getInstance().getIndividualReefTagPose(target.branch().face, getLineupPose(target));
        });

        this.leds = leds;

        this.finishSequence = finishSequence;
        this.lineupFeedback = lineupFeedback;

        addRequirements(drive);
        setName("ReefLineup");
    }

    @Override
    public void execute() {
        super.execute();

        boolean atSetpoint = atSetpoint();
        if(lineupFeedback != null) {
            lineupFeedback.accept(feedbackDebouncer.calculate(atSetpoint));
        }
        leds.setStateActive(LEDState.AutoScoreReady, ledDebouncer.calculate(atSetpoint));
    }

    @Override
    public boolean isFinished() {
        if(finishSequence.isEmpty()) {
            return atSetpoint() && RobotState.getInstance().isReefLineupSafe();
        } else {
            if(finishSequence.get().getAsBoolean()) return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        leds.setStateActive(LEDState.AutoScoreReady, false);
    }
}
