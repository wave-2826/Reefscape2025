package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

/**
 * Constants related to the arm subsystem.
 */
public class ArmConstants {
    public class ElevatorConstants {
        public static final int elevatorHeightMotor1Id = /* TODO */ 50;
        public static final int elevatorHeightMotor2Id = /* TODO */ 51;

        // PID constants for the elevator position PID
        public static final double elevatorKp = 18.0;
        public static final double elevatorKi = 0.0;
        public static final double elevatorKd = 4.0;

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

        // The PID constants for the arm pitch position PID
        public static final double armPitchKp = 0.5;
        public static final double armPitchKi = 0.0;
        public static final double armPitchKd = 0.0;

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

        // The PID constants for the arm wrist position PID
        public static final double armWristPositionKp = 0.5;
        public static final double armWristPositionKi = 0.0;
        public static final double armWristPositionKd = 0.0;

        // The PID constants for the arm wrist velocity PID
        public static final double armWristVelocityKp = 0.5;
        public static final double armWristVelocityKi = 0.0;
        public static final double armWristVelocityKd = 0.0;
        public static final double armWristVelocityKf = 1. / 917; // 917 is the Neo 550 Kf value

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

        // The PIDF constnats for the end effector velocity PID
        public static final double endEffectorKp = 0.5;
        public static final double endEffectorKi = 0.0;
        public static final double endEffectorKd = 0.0;
        // TODO: Find the proper value for this. Is it multiplied by the setpoint or motor speed? The REV docs are unclear.
        public static final double endEffectorKf = 1. / 565;

        /** The conversion factor from end effector motor rotations to radians. */
        public static final double endEffectorPositionConversionFactor = 2 * Math.PI / endEffectorReduction;
        /** The conversion factor from wrist motor RPM to radians per second. */
        public static final double endEffectorVelocityConversionFactor = endEffectorPositionConversionFactor / 60.;
        public static final int endEffectorMotorCurrentLimit = 10;
        public static final boolean endEffectorMotorInverted = false;
    }
}
