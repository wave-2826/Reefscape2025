package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim extends ArmIOReal {
    private ElevatorSim elevatorSim;
    private SingleJointedArmSim armSim;
    private DCMotorSim armWristSim;
    private DCMotorSim endEffectorSim;

    private DCMotor elevatorMotors;
    private DCMotor armMotors;
    private DCMotor armWristMotors;
    private DCMotor endEffectorMotors;

    // We only use one spark flex sim for both elevator motors.
    // Maybe there's a better way to structure this?
    private SparkFlexSim elevatorMotorSim;
    private SparkMaxSim armPitchMotorSim;
    private SparkMaxSim armWristMotorSim;
    private SparkFlexSim endEffectorMotorSim;

    private SparkLimitSwitchSim elevatorResetLimitSwitchSim;
    private SparkAbsoluteEncoderSim armPitchEncoderSim;
    private SparkAbsoluteEncoderSim armWristEncoderSim;
    private SparkLimitSwitchSim verticalGamePieceSwitchSim;
    private SparkLimitSwitchSim horizontalGamePieceSwitchSim;

    private static final double elevatorCarriageMassKg = 3.;

    /** The arm moment of inertia around its pitch axis in kg m^2. */
    private static final double armMOI = 0.5; // TODO
    private static final Rotation2d armMinAngle = Rotation2d.fromDegrees(-30.); // TODO
    private static final Rotation2d armMaxAngle = Rotation2d.fromDegrees(90.); // TODO

    /** The arm moment of inertia around its roll axis in kg m^2 */
    private static final double armWristMOI = 0.02; // TODO

    private static final double endEffectorMOI = 0.001; // TODO

    /** Whether there's currently a game piece in the end effector. */
    private boolean gamePieceInEndEffector = false;

    public ArmIOSim() {
        super();

        elevatorMotors = DCMotor.getNeoVortex(2);
        armMotors = DCMotor.getNEO(1);
        armWristMotors = DCMotor.getNeo550(1);
        endEffectorMotors = DCMotor.getNeoVortex(1);

        elevatorMotorSim = new SparkFlexSim(elevatorHeightMotor1, elevatorMotors);
        armPitchMotorSim = new SparkMaxSim(armPitchMotor, armMotors);
        armWristMotorSim = new SparkMaxSim(armWristMotor, armWristMotors);
        endEffectorMotorSim = new SparkFlexSim(endEffectorMotor, endEffectorMotors);

        elevatorResetLimitSwitchSim = elevatorMotorSim.getForwardLimitSwitchSim();
        armPitchEncoderSim = armPitchMotorSim.getAbsoluteEncoderSim();
        armWristEncoderSim = armWristMotorSim.getAbsoluteEncoderSim();
        verticalGamePieceSwitchSim = armWristMotorSim.getForwardLimitSwitchSim();
        horizontalGamePieceSwitchSim = armWristMotorSim.getReverseLimitSwitchSim();

        // Gravity simulation is turned off because the elevator and arm are counterweighted.

        elevatorSim = new ElevatorSim(elevatorMotors, ArmConstants.ElevatorConstants.elevatorReduction,
            elevatorCarriageMassKg, ArmConstants.ElevatorConstants.elevatorDrumRadiusMeters, 0,
            ArmConstants.ElevatorConstants.maxElevatorHeight.in(Meters), false, Units.inchesToMeters(15.0));

        armSim = new SingleJointedArmSim(armMotors, ArmConstants.ShoulderConstants.elevatorPitchReduction, armMOI,
            ArmConstants.ShoulderConstants.armLength.in(Meters), armMinAngle.getRadians(), armMaxAngle.getRadians(),
            true, Rotation2d.fromDegrees(0.).getRadians());

        armWristSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(armWristMotors, armWristMOI,
            ArmConstants.ShoulderConstants.armWristReduction), armWristMotors);

        endEffectorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(endEffectorMotors, endEffectorMOI,
            ArmConstants.EndEffectorConstants.endEffectorReduction), endEffectorMotors);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        // Update simulated IO on motor controllers
        if(Math.abs(
            elevatorSim.getPositionMeters() - ArmConstants.ElevatorConstants.resetSwitchHeight.in(Meters)) < 0.03) {
            elevatorResetLimitSwitchSim.setPressed(true);
        } else {
            elevatorResetLimitSwitchSim.setPressed(false);
        }

        verticalGamePieceSwitchSim.setPressed(gamePieceInEndEffector);
        horizontalGamePieceSwitchSim.setPressed(gamePieceInEndEffector);

        armPitchEncoderSim.setPosition(armSim.getAngleRads());
        armPitchEncoderSim.setVelocity(armSim.getVelocityRadPerSec());

        armWristEncoderSim.setPosition(armWristSim.getAngularPositionRad());
        armWristEncoderSim.setVelocity(armWristSim.getAngularVelocityRadPerSec());

        // TODO: Update Vin with battery simulation
        double vinVoltage = RoboRioSim.getVInVoltage();

        elevatorSim.setInputVoltage(elevatorMotorSim.getAppliedOutput() * vinVoltage);
        armSim.setInputVoltage(armPitchMotorSim.getAppliedOutput() * vinVoltage);
        armWristSim.setInputVoltage(armWristMotorSim.getAppliedOutput() * vinVoltage);
        endEffectorSim.setInputVoltage(endEffectorMotorSim.getAppliedOutput() * vinVoltage);

        elevatorSim.update(0.02);
        armSim.update(0.02);
        armWristSim.update(0.02);
        endEffectorSim.update(0.02);

        elevatorMotorSim.iterate(elevatorSim.getVelocityMetersPerSecond(), vinVoltage, 0.02);
        armPitchMotorSim.iterate(armSim.getVelocityRadPerSec(), vinVoltage, 0.02);
        armWristMotorSim.iterate(armWristSim.getAngularVelocityRadPerSec(), vinVoltage, 0.02);
        endEffectorMotorSim.iterate(endEffectorSim.getAngularVelocityRadPerSec(), vinVoltage, 0.02);

        super.updateInputs(inputs);
    }
}
