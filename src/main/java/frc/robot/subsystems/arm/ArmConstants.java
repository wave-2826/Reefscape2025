package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.LoggedTunableSparkPID;

/**
 * Constants related to the arm subsystem.
 */
public class ArmConstants {
    public class ElevatorConstants {
        public static final int elevatorHeightMotor1Id = /* TODO */ 50;
        public static final int elevatorHeightMotor2Id = /* TODO */ 51;

        // PID constants for the elevator position PID
        public static final LoggedTunableSparkPID elevatorPID = new LoggedTunableSparkPID("Arm/Elevator")
            .addRealRobotGains(18.0, 0.0, 4.0).addSimGains(18.0, 0.0, 4.0);

        public static final int elevatorMotorCurrentLimit = 30;
        public static final boolean elevatorMotorInverted = false;

        public static final double elevatorReduction = 25.;
        public static final double elevatorDrumRadiusMeters = Units.inchesToMeters(2);
        /** The conversion factor from elevator motor rotations to height in meters. */
        public static final double elevatorPositionConversionFactor = 2 * Math.PI * elevatorDrumRadiusMeters
            / elevatorReduction;

        /** The conversion factor from elevator motor RPM to velocity in meters per second. */
        public static final double elevatorVelocityConversionFactor = elevatorPositionConversionFactor / 60.;

        public static final Distance resetSwitchHeight = Meters.of(/* TODO */ 0.4);

        /**
         * The maximum height of the outer stage relative to its hard stop.
         */
        public final static Distance outerStageMaximumHeight = Inches.of(34.);
        /**
         * The maximum height of the carriage relative to the outer stage.
         */
        public final static Distance carriageMaxHeight = Inches.of(26.);
        /**
         * The maximum elevator height.
         */
        public static final Distance maxElevatorHeight = outerStageMaximumHeight.plus(carriageMaxHeight);

        /**
         * The translation from the center of the robot at the floor to the center of the elevator support structure on
         * the top of its plate.
         */
        public static final Translation3d elevatorOrigin = new Translation3d(Units.inchesToMeters(8), // Forward is +X
            0, // Left is +Y
            Units.inchesToMeters(0.65) // Upward is +Y
        );
        /**
         * The translation from the elevator origin to the outer stage when the stage is at its minimum height. The
         * stage origin is the center of the stage at the bottom of the aluminum extrusion.
         */
        public static final Translation3d elevatorToOuterStage = new Translation3d(Units.inchesToMeters(0), // Forward is +X
            0, // Left is +Y
            Units.inchesToMeters(1.81) // Upward is +Y
        );
        /**
         * The translation from the elevator origin to the carriage when the arm is at its minimum height. The carriage
         * origin is the center of the bottom plate.
         */
        public static final Translation3d elevatorToCarriage = new Translation3d(Units.inchesToMeters(0), // Forward is +X
            0, // Left is +Y
            Units.inchesToMeters(1.81) // Upward is +Y
        );
    }

    public class ShoulderConstants {
        public static final int armPitchMotorId = /* TODO */ 52;

        public static final LoggedTunableSparkPID armPitchPID = new LoggedTunableSparkPID("Arm/Pitch")
            .addRealRobotGains(0.5, 0.0, 0.0).addSimGains(0.5, 0.0, 0.0);

        public static final double elevatorPitchReduction = 5 * (84 / 48);
        /** The conversion factor from pitch motor rotations to radians. */
        public static final double pitchPositionConversionFactor = 1 / elevatorPitchReduction * 2 * Math.PI;
        /** The conversion factor from pitch motor RPM to radians per second. */
        public static final double pitchVelocityConversionFactor = pitchPositionConversionFactor / 60.;
        public static final int pitchMotorCurrentLimit = 15;
        public static final boolean pitchMotorInverted = false;

        public static final int armWristMotorId = /* TODO */ 53;

        public static final ClosedLoopSlot armWristPositionSlot = ClosedLoopSlot.kSlot0;
        public static final ClosedLoopSlot armWristVelocitySlot = ClosedLoopSlot.kSlot1;

        public static final LoggedTunableSparkPID armWristPID = new LoggedTunableSparkPID("Arm/Wrist")
            .addRealRobotGains(0.5, 0.0, 0.0, armWristPositionSlot).addSimGains(0.5, 0.0, 0.0, armWristPositionSlot)
            .addRealRobotGains(0.5, 0.0, 1 / 917, armWristVelocitySlot)
            .addSimGains(0.5, 0.0, 0.0, 1 / 917, armWristVelocitySlot); // 917 is the Neo 550 Kf value

        public static final double armWristReduction = 25.;
        /** The conversion factor from wrist motor rotations to radians. */
        public static final double wristPositionConversionFactor = 2 * Math.PI / armWristReduction;
        /** The conversion factor from wrist motor RPM to radians per second. */
        public static final double wristVelocityConversionFactor = wristPositionConversionFactor / 60.;
        public static final int wristMotorCurrentLimit = 10;
        public static final boolean wristMotorInverted = false;

        /** The translation from the carriage origin to the pivot. */
        public static final Translation3d carriageToPivot = new Translation3d(Units.inchesToMeters(3.0), // Forward is +X
            Units.inchesToMeters(-0.223), // Left is +Y. Yes, it's off-center by design.
            Units.inchesToMeters(6.0) // Upward is +Y
        );
        /** The translation from the pivot to the end effector. This must be rotated by the arm pitch. */
        public static final Translation3d pivotToEndEffector = new Translation3d(Units.inchesToMeters(16.44), // Forward is +X
            Units.inchesToMeters(0.0), // Left is +Y
            Units.inchesToMeters(1.11) // Upward is +Y
        );

        /** The arm length. */
        public static final Distance armLength = Meters.of(Math.sqrt(Math.pow(pivotToEndEffector.getX(), 2)
            + Math.pow(pivotToEndEffector.getY(), 2) + Math.pow(pivotToEndEffector.getZ(), 2)));
    }

    public class EndEffectorConstants {
        public static final int endEffectorMotorId = /* TODO */ 54;

        public static final double endEffectorReduction = 5.;

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
        public static final double endEffectorCouplingFactor = 1.0; // TODO: Find the proper value for this. Maybe from empirical testing?

        // TODO: Find the proper value for this. Is it multiplied by the setpoint or motor speed? The REV docs are unclear.
        public static final LoggedTunableSparkPID endEffectorPID = new LoggedTunableSparkPID("Arm/EndEffector")
            .addRealRobotGains(0.5, 0.0, 0.0, 1. / 565, endEffectorVelocitySlot)
            .addSimGains(0.5, 0.0, 0.0, 1. / 565, endEffectorVelocitySlot)
            .addRealRobotGains(0.5, 0.0, 0.0, endEffectorPositionSlot)
            .addSimGains(0.5, 0.0, 0.0, endEffectorPositionSlot);

        /** The conversion factor from end effector motor rotations to radians. */
        public static final double endEffectorPositionConversionFactor = 2 * Math.PI / endEffectorReduction;
        /** The conversion factor from wrist motor RPM to radians per second. */
        public static final double endEffectorVelocityConversionFactor = endEffectorPositionConversionFactor / 60.;
        public static final int endEffectorMotorCurrentLimit = 10;
        public static final boolean endEffectorMotorInverted = false;
    }
}
