package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.vision.VisionConstants;

/**
 * Field-related constants for the 2025 game. All measurements are by default relative to the blue alliance.
 */
public class FieldConstants {
    /** The field length in meters. */
    public static final double fieldLength = VisionConstants.aprilTagLayout.getFieldLength();
    /** The field width in meters. */
    public static final double fieldWidth = VisionConstants.aprilTagLayout.getFieldWidth();

    public static final double algaeDiameter = Units.inchesToMeters(16);
    public static final double coralDiameter = Units.inchesToMeters(4.5);

    public enum ReefLevel {
        // @formatter:off
        L4(Inches.of(72), Degrees.of(-90)),
        L3(Inches.of(47.625), Degrees.of(-35)),
        L2(Inches.of(31.875), Degrees.of(-35)),
        L1(Inches.of(18), Degrees.of(0));
        // @formatter:on

        public final Distance height;
        /**
         * The angle of this reef position. Negative values are upward.
         */
        public final Angle pitch;

        ReefLevel(Distance height, Angle pitch) {
            this.height = height;
            this.pitch = pitch; // in degrees
        }
    }

    public static final Distance reefBranchSeparation = Inches.of(13.);
    public static final Distance reefBranchInset = Inches.of(12.066);

    public static final Translation2d reefCenter = new Translation2d(Units.inchesToMeters(176.746), fieldWidth / 2.);

    /**
     * A reef branch, as labelled by the FMS. See
     * https://firstfrc.blob.core.windows.net/frc2025/Manual/HTML/2025GameManual_files/image016.png
     */
    public enum ReefBranch {
        /** The left branch of the face closest to the driver station. */
        A(ReefFace.Front, true),
        /** The right branch of the face closest to the driver station. */
        B(ReefFace.Front, false),
        /** The left branch of the face on the right when at driver stations. */
        C(ReefFace.FrontRight, true),
        /** The right branch of the face on the right when at driver stations. */
        D(ReefFace.FrontRight, false),
        /** The left branch of the face closest to the processor. */
        E(ReefFace.BackRight, true),
        /** The right branch of the face closest to the processor. */
        F(ReefFace.BackRight, false),
        /** The left branch of the face furthest from the driver station. */
        G(ReefFace.Back, true),
        /** The right branch of the face furthest from the driver station. */
        H(ReefFace.Back, false),
        /** The left branch of the face on the back left when at driver stations. */
        I(ReefFace.BackLeft, true),
        /** The right branch of the face on the back left when at driver stations. */
        J(ReefFace.BackLeft, false),
        /** The left branch of the face on the left when at driver stations. */
        K(ReefFace.FrontLeft, true),
        /** The right branch of the face on the left when at driver stations. */
        L(ReefFace.FrontLeft, false);

        public Pose2d pose;
        public ReefFace face;
        public boolean isLeft;

        ReefBranch(ReefFace face, boolean left) {
            this.pose = left ? face.getLeftBranchPose() : face.getRightBranchPose();
            this.face = face;
            this.isLeft = left;
        }
    }

    /**
     * A reef face. Names are relative to the driver station.
     */
    public enum ReefFace {
        // @formatter:off
        Front(18, 7),
        FrontLeft(19, 6),
        BackLeft(20, 11),
        Back(21, 10),
        BackRight(22, 9),
        FrontRight(17, 8);
        // @formatter:on

        public final Pose2d blueTagPose;
        public final Pose2d redTagPose;

        public final int aprilTagBlue;
        public final int aprilTagRed;

        ReefFace(int blueAprilTag, int redAprilTag) {
            this.blueTagPose = VisionConstants.aprilTagLayout.getTagPose(blueAprilTag).get().toPose2d();
            this.redTagPose = VisionConstants.aprilTagLayout.getTagPose(redAprilTag).get().toPose2d();

            this.aprilTagBlue = blueAprilTag;
            this.aprilTagRed = redAprilTag;
        }

        /**
         * Gets the angle that the robot should move at when lining up with this face for the given alliance -- the
         * angle facing into it.
         */
        public Rotation2d getFieldAngle(boolean isRed) {
            var tagPose = isRed ? redTagPose : blueTagPose;
            return tagPose.getRotation().plus(Rotation2d.k180deg);
        }

        /**
         * Gets the angle that the robot should move at when lining up with this face for the current alliance -- the
         * angle facing into it.
         */
        public Rotation2d getFieldAngle() {
            return getFieldAngle(DriverStation.getAlliance().orElse(Alliance.Blue) == DriverStation.Alliance.Red);
        }

        /**
         * Gets the AprilTag pose for a given alliance.
         * @param isRed
         * @return
         */
        public Pose2d getTagPose(boolean isRed) {
            return isRed ? redTagPose : blueTagPose;
        }

        /**
         * Gets the AprilTag pose for the current alliance.
         * @return
         */
        public Pose2d getTagPose() {
            return getTagPose(DriverStation.getAlliance().orElse(Alliance.Blue) == DriverStation.Alliance.Red);
        }

        /** Gets the AprilTag ID for a given alliance. */
        public int getAprilTagID(boolean isRed) {
            return isRed ? aprilTagRed : aprilTagBlue;
        }

        /** Gets the AprilTag ID for the current alliance. */
        public int getAprilTagID() {
            return getAprilTagID(DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red);
        }

        public Pose2d getLeftBranchPose() {
            return blueTagPose.transformBy(
                new Transform2d(-reefBranchInset.in(Meters), -reefBranchSeparation.in(Meters) / 2., Rotation2d.kZero));
        }

        public Pose2d getRightBranchPose() {
            return blueTagPose.transformBy(
                new Transform2d(-reefBranchInset.in(Meters), reefBranchSeparation.in(Meters) / 2., Rotation2d.kZero));
        }
    }
}
