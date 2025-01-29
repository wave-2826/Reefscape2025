package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.config.ClosedLoopConfig;

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

        public static final Distance elevatorResetSwitchHeight = Meters.of(/* TODO */ 1.5);
    }

    public class ArmPitchConstants {
        public static final int armPitchMotorId = /* TODO */ 52;
        public static final ClosedLoopConfig elevatorPitchClosedLoopConfig = // Position
            new ClosedLoopConfig().pid(/* TODO */ 0.5, 0.0, 0.0);
    }

    public class ArmWristConstants {
        public static final int armWristMotorId = /* TODO */ 53;
        public static final ClosedLoopConfig armWristClosedLoopConfig = // Position
            new ClosedLoopConfig().pid(/* TODO */ 0.5, 0.0, 0.0);
    }

    public class EndEffectorConstants {
        public static final int endEffectorMotorId = /* TODO */ 54;
        public static final ClosedLoopConfig endEffectorClosedLoopConfig = // Velocity
            new ClosedLoopConfig().pid(/* TODO */ 0.5, 0.0, 0.0);
    }
}
