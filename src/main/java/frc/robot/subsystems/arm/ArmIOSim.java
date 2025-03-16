package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.sim.LaserCanSim;

public class ArmIOSim extends ArmIOReal {
    private final ElevatorSim elevatorSim;
    private final SingleJointedArmSim armSim;
    private final DCMotorSim armWristSim;
    private final DCMotorSim endEffectorSim;

    private final DCMotor elevatorMotors;
    private final DCMotor armMotors;
    private final DCMotor armWristMotors;
    private final DCMotor endEffectorMotors;

    // We only use one spark sim for both elevator motors.
    // Maybe there's a better way to structure this?
    private final SparkMaxSim elevatorMotorSim;
    private final SparkMaxSim armPitchMotorSim;
    private final SparkMaxSim armWristMotorSim;
    private final SparkMaxSim endEffectorMotorSim;

    private final SparkAbsoluteEncoderSim armPitchEncoderSim;
    private final SparkAbsoluteEncoderSim armWristEncoderSim;

    private static final double elevatorCarriageMassKg = 5.9;

    /** The arm moment of inertia around its pitch axis in kg m^2. */
    private static final double armMOI = 0.162;
    private static final Rotation2d armMinAngle = Rotation2d.fromDegrees(-90.);
    private static final Rotation2d armMaxAngle = Rotation2d.fromDegrees(90.);

    /** The arm moment of inertia around its roll axis in kg m^2 */
    private static final double armWristMOI = 0.0051;
    private static final double endEffectorMOI = 0.0001;

    /** Whether there's currently a game piece in the end effector. */
    private boolean gamePieceInEndEffector = false;

    public ArmIOSim() {
        super(
            // Override the elevator height sensor with a simulated one.
            new LaserCanSim(ArmConstants.ElevatorConstants.elevatorHeightSensorId));

        elevatorMotors = DCMotor.getNeoVortex(2);
        armMotors = DCMotor.getNEO(1);
        armWristMotors = DCMotor.getNeo550(1);
        endEffectorMotors = DCMotor.getNeoVortex(1);

        elevatorMotorSim = new SparkMaxSim(elevatorHeightMotorLeader, DCMotor.getNeoVortex(1));
        armPitchMotorSim = new SparkMaxSim(armPitchMotor, armMotors);
        armWristMotorSim = new SparkMaxSim(armWristMotor, armWristMotors);
        endEffectorMotorSim = new SparkMaxSim(endEffectorMotor, endEffectorMotors);

        armPitchEncoderSim = armPitchMotorSim.getAbsoluteEncoderSim();
        armWristEncoderSim = armWristMotorSim.getAbsoluteEncoderSim();

        elevatorSim = new ElevatorSim(elevatorMotors, ArmConstants.ElevatorConstants.elevatorReduction,
            elevatorCarriageMassKg, ArmConstants.ElevatorConstants.elevatorDrumRadiusMeters, 0,
            ArmConstants.ElevatorConstants.maxElevatorHeight.in(Meters), true, Units.inchesToMeters(15.0));

        armSim = new SingleJointedArmSim(armMotors, ArmConstants.ShoulderConstants.armPitchReduction, armMOI,
            ArmConstants.ShoulderConstants.armLength.in(Meters), armMinAngle.getRadians(), armMaxAngle.getRadians(),
            true, Rotation2d.fromDegrees(0.).getRadians());

        armWristSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(armWristMotors, armWristMOI,
            ArmConstants.ShoulderConstants.armWristReduction), armWristMotors);

        endEffectorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(endEffectorMotors, endEffectorMOI,
            ArmConstants.EndEffectorConstants.endEffectorReduction), endEffectorMotors);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        // TODO: Update Vin with battery simulation
        double vinVoltage = RoboRioSim.getVInVoltage();

        double simDt = 0.002;

        for(double t = 0; t < 0.02; t += simDt) {
            elevatorSim.setInputVoltage(elevatorMotorSim.getAppliedOutput() * vinVoltage);
            armSim.setInputVoltage(armPitchMotorSim.getAppliedOutput() * vinVoltage);
            armWristSim.setInputVoltage(armWristMotorSim.getAppliedOutput() * vinVoltage);
            endEffectorSim.setInputVoltage(endEffectorMotorSim.getAppliedOutput() * vinVoltage);

            elevatorSim.update(simDt);
            armSim.update(simDt);
            armWristSim.update(simDt);
            endEffectorSim.update(simDt);

            armPitchEncoderSim.setPosition(armSim.getAngleRads());
            armPitchEncoderSim.setVelocity(armSim.getVelocityRadPerSec());

            armWristEncoderSim.setPosition(armWristSim.getAngularPositionRad());
            armWristEncoderSim.setVelocity(armWristSim.getAngularVelocityRadPerSec());

            elevatorMotorSim.iterate(elevatorSim.getVelocityMetersPerSecond(), vinVoltage, simDt);

            armPitchMotorSim.iterate(armSim.getVelocityRadPerSec(), vinVoltage, simDt);
            armWristMotorSim.iterate(armWristSim.getAngularVelocityRadPerSec(), vinVoltage, simDt);
            endEffectorMotorSim.iterate(endEffectorSim.getAngularVelocityRadPerSec(), vinVoltage, simDt);
        }

        ((LaserCanSim) elevatorHeightSensor).setMeasurement(0, (int) (elevatorSim.getPositionMeters() * 1000), 0);

        super.updateInputs(inputs);

        // We normally average the leader and followers' encoder readings, but the encoder on the follower
        // will measure 0 in simulation so this gives incorrect results. This fixes that.
        inputs.elevatorHeightMeters = leaderElevatorHeightEncoder.getPosition();
        inputs.elevatorVelocityMetersPerSecond = leaderElevatorHeightEncoder.getVelocity();

        // For some reason, normal encoder resetting doens't work with simulated sparks. We do this in ArmIOSim instead.
        if(needsToReset) {
            elevatorMotorSim.setPosition(elevatorSim.getPositionMeters());
            needsToReset = false;
        }

        inputs.gamePiecePresent = gamePieceInEndEffector;
    }
}
