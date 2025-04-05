package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.arm.ArmState.WristRotation;
import frc.robot.util.LoggedTunableSparkPID;
import frc.robot.util.PIDConstants;
import frc.robot.util.GearRatios.UltraPlanetaryRatio;

/**
 * Constants related to the arm subsystem.
 */
public class ArmConstants {
    /** The state that the arm rests in while waiting for a game piece. */
    public static final ArmState restingState = new ArmState(Rotation2d.fromDegrees(-102), Meters.of(0.5),
        WristRotation.Horizontal, EndEffectorState.hold());
    /** The state the arm is in when getting a piece. */
    public static final ArmState getPieceState = new ArmState(Rotation2d.fromDegrees(-102), Meters.of(0.3),
        WristRotation.Horizontal, EndEffectorState.velocity(-20));

    /** The state when the arm is intaking from the source. */
    public static final ArmState sourceIntakeState = new ArmState(Rotation2d.fromDegrees(75), Inches.of(4),
        WristRotation.Vertical, EndEffectorState.velocity(-20));
    /** The state when the arm is intaking from the source but stopped. */
    public static final ArmState sourceIntakeStoppedState = new ArmState(Rotation2d.fromDegrees(75), Inches.of(4),
        WristRotation.Vertical, EndEffectorState.hold());

    /** The state when the arm is preparing for scoring. */
    public static final ArmState prepForScoringState = new ArmState(Rotation2d.fromDegrees(80), restingState.height(),
        WristRotation.HorizontalFlipped, EndEffectorState.hold());

    public class ElevatorConstants {
        public static final double elevatorStartingHeightMeters = 0.34;
        public static final double bottomResetHeightMeters = 0.085;

        public static final int elevatorHeightMotor1Id = 50;
        public static final int elevatorHeightMotor2Id = 51;
        public static final int elevatorHeightSensorId = 54;

        // PID constants for the elevator position PID
        public static final LoggedTunableSparkPID elevatorPID = new LoggedTunableSparkPID("Arm/Elevator")
            // .addRealRobotGains(new PIDConstants(7.0, 0.0, 7.0)) //
            .addRealRobotGains(new PIDConstants(7.0, 0.005, 7.0).iZone(Units.inchesToMeters(1.5))) //
            .addSimGains(new PIDConstants(40.0, 0.0, 0.0));

        // Feedforward constants for the elevator
        /** The gravity gain in volts. */
        public static final double elevatorKgReal = 0.3;
        public static final double elevatorKgSim = 0.3;

        public static final int elevatorMotorCurrentLimit = 45;
        public static final boolean elevatorMotorInverted = true;

        public static final double elevatorReduction = 5.;
        public static final double elevatorDrumRadiusMeters = Units.inchesToMeters(1.88 / 2.);
        /** The conversion factor from elevator motor rotations to height in meters. */
        public static final double elevatorPositionConversionFactor = 2 * Math.PI * elevatorDrumRadiusMeters
            / elevatorReduction;

        /** The conversion factor from elevator motor RPM to velocity in meters per second. */
        public static final double elevatorVelocityConversionFactor = elevatorPositionConversionFactor / 60.;

        /**
         * The maximum height of the outer stage relative to its hard stop.
         */
        public final static Distance outerStageMaximumHeight = Inches.of(34.);
        /**
         * The maximum height of the carriage relative to the outer stage.
         */
        public final static Distance carriageMaxHeight = Inches.of(26.);
        /**
         * The height of the carriage.
         */
        public final static Distance carriageHeight = Inches.of(14.);
        /**
         * The maximum elevator height.
         */
        public static final Distance maxElevatorHeight = outerStageMaximumHeight.plus(carriageMaxHeight);

        /**
         * The margin on the elevator bottom soft stop.
         */
        public static final Distance softStopMarginBottom = Inches.of(2.0);
        /**
         * The margin on the elevator top soft stop.
         */
        public static final Distance softStopMarginTop = Inches.of(5.5);

        /**
         * The translation from the center of the robot at the floor to the center of the elevator support structure on
         * the top of its plate.
         */
        public static final Translation3d elevatorOrigin = new Translation3d(Units.inchesToMeters(8), // Forward is +X
            0, // Left is +Y
            Units.inchesToMeters(2.0) // Upward is +Y
        );
        /**
         * The translation from the elevator origin to the passive (outer) stage when the stage is at its minimum
         * height.
         */
        public static final Translation3d elevatorToPassiveStage = new Translation3d(Units.inchesToMeters(0), // Forward is +X
            0, // Left is +Y
            Units.inchesToMeters(0.0) // Upward is +Y
        );
        /**
         * The translation from the elevator origin to the carriage when the arm is at its minimum height. The carriage
         * origin is the center of the bottom plate.
         */
        public static final Translation3d elevatorToCarriage = new Translation3d(Units.inchesToMeters(0), // Forward is +X
            0, // Left is +Y
            Units.inchesToMeters(0.0) // Upward is +Y
        );
    }

    public class ShoulderConstants {
        public static final int armPitchMotorId = 52;
        public static final int armWristMotorId = 53;

        /** The wrist absolute encoder zero offset, in radians. */
        public static final double wristZeroOffset = 0.1882066;
        /** The pitch absolute encoder zero offset, in radians. */
        public static final double pitchZeroOffset = 0.3511382;

        /** The highest soft stop for the arm pitch. */
        public static final Rotation2d maximumPitch = Rotation2d.fromDegrees(90.);
        /** The lowest soft stop for the arm pitch. */
        public static final Rotation2d minimumPitch = Rotation2d.fromDegrees(-100.);

        /** The feedforward gravity constant for the arm in volts. */
        public static final double armPitchKgReal = 0.50;
        public static final double armPitchKgSim = 0.50;

        public static final LoggedTunableSparkPID armPitchPID = new LoggedTunableSparkPID("Arm/Pitch")
            // .addRealRobotGains(new PIDConstants(0.8, 0.001, 0.9).iZone(Units.degreesToRadians(5)))
            .addRealRobotGains(new PIDConstants(0.5, 0.001, 0.8).iZone(Units.degreesToRadians(5)))
            .addSimGains(new PIDConstants(25., 0.001, 0.3).iZone(Units.degreesToRadians(5)));

        public static final double armPitchReduction = 9 * (36 / 12);
        /** The conversion factor from pitch motor rotations to radians. */
        public static final double pitchPositionConversionFactor = 1 / armPitchReduction * 2 * Math.PI;
        /** The conversion factor from pitch motor RPM to radians per second. */
        public static final double pitchVelocityConversionFactor = pitchPositionConversionFactor / 60.;

        /** The conversion factor from pitch absolute encoder to radians. */
        public static final double pitchAbsolutePositionFactor = 2 * Math.PI;
        /** The conversion factor from pitch absolute encoder RPM to radians per second. */
        public static final double pitchAbsoluteVelocityFactor = pitchAbsolutePositionFactor / 60.;

        public static final int pitchMotorCurrentLimit = 38;
        public static final boolean pitchMotorInverted = true;
        public static final boolean pitchEncoderInverted = true;

        public static final ClosedLoopSlot armWristPositionSlot = ClosedLoopSlot.kSlot0;
        public static final ClosedLoopSlot armWristVelocitySlot = ClosedLoopSlot.kSlot1;

        public static final LoggedTunableSparkPID armWristPID = new LoggedTunableSparkPID("Arm/Wrist")
            // .addRealRobotGains(new PIDConstants(0.85, 0.0, 14.0, armWristPositionSlot)) //
            .addRealRobotGains(new PIDConstants(0.6, 0.005, 6.0, armWristPositionSlot).iZone(Units.degreesToRadians(5))) //
            .addSimGains(new PIDConstants(0.05, 0.0, 0.0, armWristPositionSlot)) //
            .addRealRobotGains(new PIDConstants(2.0, 0.0, 1 / 917, armWristVelocitySlot))
            .addSimGains(new PIDConstants(2.0, 0.0, 0.0, 1 / 917, armWristVelocitySlot)); // 917 is the Neo 550 Kf value

        public static final double armWristReduction = UltraPlanetaryRatio.FIVE_TO_ONE.ratio
            * UltraPlanetaryRatio.FIVE_TO_ONE.ratio;
        /** The conversion factor from wrist motor rotations to radians. */
        public static final double wristPositionConversionFactor = 2 * Math.PI / armWristReduction;
        /** The conversion factor from wrist motor RPM to radians per second. */
        public static final double wristVelocityConversionFactor = wristPositionConversionFactor / 60.;

        /** The conversion factor from wrist absolute encoder rotations to radians. */
        public static final double wristAbsolutePositionFactor = 2 * Math.PI;
        /** The conversion factor from wrist absolute encoder RPM to radians per second. */
        public static final double wristAbsoluteVelocityFactor = wristPositionConversionFactor / 60.;

        public static final int wristMotorCurrentLimit = 20;
        public static final boolean wristMotorInverted = false;

        /** The translation from the carriage origin to the shoulder. */
        public static final Translation3d carriageToShoulder = new Translation3d(Units.inchesToMeters(4.5), // Forward is +X
            Units.inchesToMeters(0.0), // Left is +Y
            Units.inchesToMeters(7.25) // Upward is +Y
        );
        /** The translation from the shoulder to the wrist. */
        public static final Translation3d shoulderToWrist = new Translation3d(Units.inchesToMeters(0.0), // Forward is +X
            Units.inchesToMeters(0.0), // Left is +Y
            Units.inchesToMeters(1.25) // Upward is +Y
        );
        /** The transform from the wrist to the coral in the end effector. */
        public static final Transform3d wristToCoral = new Transform3d(new Translation3d(Units.inchesToMeters(4.5), // Forward is +X
            Units.inchesToMeters(0.0), // Left is +Y
            Units.inchesToMeters(16.1) // Upward is +Y
        ), new Rotation3d(Units.degreesToRadians(0.0), // Roll
            Units.degreesToRadians(0.0), // Pitch
            Units.degreesToRadians(0.0) // Yaw
        ));

        /** The arm length. */
        public static final Distance armLength = Inches.of(16.1);
    }

    public class EndEffectorConstants {
        public static final int endEffectorMotorId = 54;

        public static final double endEffectorReduction = 24. / 11. * 9.;

        // The PID slot used for end effector position control. We need to use position control because
        // the coaxial drive system has a coupling factor that could cause the piece to be spit out if
        // the wrist is spun.
        public static final ClosedLoopSlot endEffectorPositionSlot = ClosedLoopSlot.kSlot0;
        // The PID slot used for end effector velocity control.
        public static final ClosedLoopSlot endEffectorVelocitySlot = ClosedLoopSlot.kSlot1;

        // The coupling factor between wrist rotation and end effector wheel rotation. Because we use
        // a coaxial drive system, the end effector wheel's rotation is linked to the wrist rotation.
        // We need to compensate for this by using position control on the end effector motor.
        // For every 1 rotation of the wrist, the end effector wheel rotates endEffectorCouplingFactor times.
        public static final double endEffectorCouplingFactor = -11. / 24.; // TODO: Find the proper value for this. Maybe from empirical testing?

        public static final LoggedTunableSparkPID endEffectorPID = new LoggedTunableSparkPID("Arm/EndEffector")
            .addRealRobotGains(new PIDConstants(0.001, 0.0, 0.5, endEffectorReduction / 565, endEffectorVelocitySlot))
            .addSimGains(new PIDConstants(0.01, 0.0, 0.0, 0.04, endEffectorVelocitySlot))
            .addRealRobotGains(new PIDConstants(0.9, 0.0, 0.1, endEffectorPositionSlot))
            .addSimGains(new PIDConstants(0.9, 0.0, 0.1, endEffectorPositionSlot));

        /** The conversion factor from end effector motor rotations to radians. */
        public static final double endEffectorPositionConversionFactor = 2 * Math.PI / endEffectorReduction;
        /** The conversion factor from wrist motor RPM to radians per second. */
        public static final double endEffectorVelocityConversionFactor = endEffectorPositionConversionFactor / 60.;
        public static final int endEffectorMotorCurrentLimit = 40;
        public static final boolean endEffectorMotorInverted = true;
    }
}
