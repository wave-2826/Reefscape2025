package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.config.ClosedLoopConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

/**
 * Constants related to the arm subsystem.
 */
public class ArmConstants {
    public class ElevatorConstants {
        public static final int elevatorHeightMotor1Id = /* TODO */ 50;
        public static final int elevatorHeightMotor2Id = /* TODO */ 51;
        public static final ClosedLoopConfig elevatorHeightClosedLoopConfig = // Position
            new ClosedLoopConfig().pid(/* TODO */ 0.5, 0.0, 0.0);
        public static final int elevatorMotorCurrentLimit = 20;
        public static final boolean elevatorMotorInverted = false;
        /** The conversion factor from elevator motor rotations to height in meters. */
        public static final double elevatorPositionConversionFactor = Math.PI * Units.inchesToMeters(2) * (1 / 5);
        /** The conversion factor from elevator motor RPM to velocity in meters per second. */
        public static final double elevatorVelocityConversionFactor = elevatorPositionConversionFactor / 60.;

        public static final Distance resetSwitchHeight = Meters.of(/* TODO */ 1.5);
    }

    public class PitchWristConstants {
        public static final int armPitchMotorId = /* TODO */ 52;
        public static final ClosedLoopConfig armPitchClosedLoopConfig = // Position
            new ClosedLoopConfig().pid(/* TODO */ 0.5, 0.0, 0.0);

        /** The conversion factor from pitch motor rotations to radians. */
        public static final double pitchPositionConversionFactor = (1 / 5) * (48 / 84) * 2 * Math.PI;
        /** The conversion factor from pitch motor RPM to radians per second. */
        public static final double pitchVelocityConversionFactor = pitchPositionConversionFactor / 60.;
        public static final int pitchMotorCurrentLimit = 15;
        public static final boolean pitchMotorInverted = false;

        public static final int armWristMotorId = /* TODO */ 53;
        public static final ClosedLoopConfig armWristClosedLoopConfig = // Position
            new ClosedLoopConfig().pid(/* TODO */ 0.5, 0.0, 0.0);

        /** The conversion factor from wrist motor rotations to radians. */
        public static final double wristPositionConversionFactor = (1 / 25) * 2 * Math.PI;
        /** The conversion factor from wrist motor RPM to radians per second. */
        public static final double wristVelocityConversionFactor = wristPositionConversionFactor / 60.;
        public static final int wristMotorCurrentLimit = 10;
        public static final boolean wristMotorInverted = false;
    }

    public class EndEffectorConstants {
        public static final int endEffectorMotorId = /* TODO */ 54;
        public static final ClosedLoopConfig endEffectorClosedLoopConfig = // Velocity
            new ClosedLoopConfig().pid(/* TODO */ 0.5, 0.0, 0.0);

        /** The conversion factor from end effector motor rotations to radians. */
        public static final double endEffectorPositionConversionFactor = (1 / 5) * 2 * Math.PI;
        /** The conversion factor from wrist motor RPM to radians per second. */
        public static final double endEffectorVelocityConversionFactor = endEffectorPositionConversionFactor / 60.;
        public static final int endEffectorMotorCurrentLimit = 10;
        public static final boolean endEffectorMotorInverted = false;
    }
}
