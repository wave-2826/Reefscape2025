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

/**
 * Field-related constants for the 2025 game. All measurements are by default relative to the blue alliance.
 */
public class FieldConstants {
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

    public static final Distance reefBranchSeparation = Inches.of(12.938);
    public static final Translation2d reefCenter = new Translation2d(Units.inchesToMeters(176.746),
        Units.inchesToMeters(158.501));

    /**
     * A reef branch, as labelled by the FMS. See
     * https://firstfrc.blob.core.windows.net/frc2025/Manual/HTML/2025GameManual_files/image016.png
     */
    public enum ReefBranch {
        /** The left branch of the face closest to the driver station. */
        A(ReefFace.Front.getLeftBranchPose()),
        /** The right branch of the face closest to the driver station. */
        B(ReefFace.Front.getRightBranchPose()),
        /** The left branch of the face on the right when at driver stations. */
        C(ReefFace.FrontRight.getLeftBranchPose()),
        /** The right branch of the face on the right when at driver stations. */
        D(ReefFace.FrontRight.getRightBranchPose()),
        /** The left branch of the face closest to the processor. */
        E(ReefFace.BackRight.getLeftBranchPose()),
        /** The right branch of the face closest to the processor. */
        F(ReefFace.BackRight.getRightBranchPose()),
        /** The left branch of the face furthest from the driver station. */
        G(ReefFace.Back.getLeftBranchPose()),
        /** The right branch of the face furthest from the driver station. */
        H(ReefFace.Back.getRightBranchPose()),
        /** The left branch of the face on the back left when at driver stations. */
        I(ReefFace.BackLeft.getLeftBranchPose()),
        /** The right branch of the face on the back left when at driver stations. */
        J(ReefFace.BackLeft.getRightBranchPose()),
        /** The left branch of the face on the left when at driver stations. */
        K(ReefFace.FrontLeft.getLeftBranchPose()),
        /** The right branch of the face on the left when at driver stations. */
        L(ReefFace.FrontLeft.getRightBranchPose());

        public Pose2d pose;

        ReefBranch(Pose2d pose) {
            this.pose = pose;
        }
    }

    /**
     * A reef face. Names are relative to the driver station.
     */
    public enum ReefFace {
        // @formatter:off
        Front(new Pose2d(
            Units.inchesToMeters(144.003),
            Units.inchesToMeters(158.500),
            Rotation2d.fromDegrees(180))),
        FrontLeft(new Pose2d(
            Units.inchesToMeters(160.373),
            Units.inchesToMeters(186.857),
            Rotation2d.fromDegrees(120))),
        BackLeft(new Pose2d(
            Units.inchesToMeters(193.116),
            Units.inchesToMeters(186.858),
            Rotation2d.fromDegrees(60))),
        Back(new Pose2d(
            Units.inchesToMeters(209.489),
            Units.inchesToMeters(158.502),
            Rotation2d.fromDegrees(0))),
        BackRight(new Pose2d(
            Units.inchesToMeters(193.118),
            Units.inchesToMeters(130.145),
            Rotation2d.fromDegrees(-60))),
        FrontRight(new Pose2d(
            Units.inchesToMeters(160.375),
            Units.inchesToMeters(130.144),
            Rotation2d.fromDegrees(-120)));
        // @formatter:on

        public final Pose2d pose;

        ReefFace(Pose2d pose) {
            this.pose = pose;
        }

        public Pose2d getLeftBranchPose() {
            return pose.transformBy(new Transform2d(0., -reefBranchSeparation.in(Meters) / 2., new Rotation2d()));
        }

        public Pose2d getRightBranchPose() {
            return pose.transformBy(new Transform2d(0., reefBranchSeparation.in(Meters) / 2., new Rotation2d()));
        }
    }
}
