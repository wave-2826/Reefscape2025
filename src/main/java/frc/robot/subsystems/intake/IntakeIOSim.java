package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.sim.SimRobotGamePiece;

public class IntakeIOSim extends IntakeIOReal {
    /** The maximum pitch above the ground that allows us to intake a game piece. */
    private static final double MAX_INTAKE_ACTIVATION_PITCH = 10.;
    /** The minimum intake wheel speed that allows us to intake, in radians per second. */
    private static final double MIN_INTAKE_WHEEL_SPEED = 157.;

    private final SingleJointedArmSim pitchSim;
    private final DCMotorSim powerSim;
    private final DCMotorSim transportSim;

    private final SparkMaxSim transportSparkSim;
    private final SparkMaxSim pitchSparkSim;
    private final SparkFlexSim powerSparkSim;

    private final SparkAbsoluteEncoderSim pitchEncoderSim;

    private static final double intakeMOI = 0.1;
    /** The intake minimum (lowest) angle. Positive numbers are downward. */
    private static final Rotation2d intakeMinAngle = Rotation2d.fromDegrees(0);
    /** The intake maximum (highest) angle. Positive numbers are downward. */
    private static final Rotation2d intakeMaxAngle = Rotation2d.fromDegrees(-90);
    private static final double intakePitchGearing = 24. * (111. / 28.);
    private static final double intakePowerGearing = 9.;
    private static final double intakePowerMOI = 0.005;
    private static final double transportMOI = 0.005;
    private static final double transportGearing = 15.;

    private final IntakeSimulation intakeSimulation;

    private final DIOSim intakeDIOSim;
    private final DIOSim transportDIOSim;

    /**
     * The simulated coral position, from 0 to 1, as it moves through the intake and transport. Null if we don't
     * currently have a game piece.
     */
    private static Double simulatedCoralPosition = null;

    /** Returns true and clears the simulated coral in the intake if one is present. */
    public static boolean takeCoral() {
        if(simulatedCoralPosition == null) return false;
        simulatedCoralPosition = null;
        return true;
    }

    public IntakeIOSim(SwerveDriveSimulation driveTrainSimulation) {
        super();

        transportSparkSim = new SparkMaxSim(transportMotor, IntakeConstants.transportMotor);
        pitchSparkSim = new SparkMaxSim(pitchMotor, IntakeConstants.intakePitchMotor);
        powerSparkSim = new SparkFlexSim(powerMotor, IntakeConstants.intakePowerMotor);

        pitchEncoderSim = pitchSparkSim.getAbsoluteEncoderSim();

        // TODO: This doesn't work for some reason now? I can't figure it out for now.
        pitchSim = new SingleJointedArmSim(IntakeConstants.intakePitchMotor, intakePitchGearing, intakeMOI,
            IntakeConstants.intakeLength.in(Meters) / 5., intakeMaxAngle.getRadians(), intakeMinAngle.getRadians(),
            true, intakeMaxAngle.getRadians());

        pitchEncoderSim.setPosition(pitchSim.getAngleRads());

        intakeDIOSim = new DIOSim(intakeSensor);
        transportDIOSim = new DIOSim(endSensor);

        powerSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(IntakeConstants.intakePowerMotor, intakePowerMOI, intakePowerGearing),
            IntakeConstants.intakePowerMotor);
        transportSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(IntakeConstants.transportMotor, transportMOI, transportGearing),
            IntakeConstants.transportMotor);

        intakeSimulation = IntakeSimulation.OverTheBumperIntake("Coral", driveTrainSimulation,
            // Width of the intake
            Inches.of(17.5),
            // The extension length of the intake beyond the robot's frame when activated
            Inches.of(9.0), IntakeSimulation.IntakeSide.BACK, 1);
    }

    public static void addCoral() {
        simulatedCoralPosition = 0.;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // TODO: Update Vin with battery simulation
        double vinVoltage = RoboRioSim.getVInVoltage();

        double simDt = 0.002;

        for(double t = 0; t < 0.02; t += simDt) {
            pitchSim.setInputVoltage(pitchSparkSim.getAppliedOutput() * vinVoltage);
            powerSim.setInputVoltage(powerSparkSim.getAppliedOutput() * vinVoltage);
            transportSim.setInputVoltage(transportSparkSim.getAppliedOutput() * vinVoltage);

            pitchSim.update(simDt);
            powerSim.update(simDt);
            transportSim.update(simDt);

            pitchEncoderSim.setPosition(pitchSim.getAngleRads());

            pitchSparkSim.iterate(pitchSim.getVelocityRadPerSec(), vinVoltage, simDt);
            powerSparkSim.iterate(powerSim.getAngularVelocityRadPerSec() * intakePowerGearing, vinVoltage, simDt);
            transportSparkSim.iterate(transportSim.getAngularVelocityRadPerSec() * transportGearing, vinVoltage, simDt);
        }

        // TODO: Track coral path and activate sensors
        intakeDIOSim.setValue(
            !(simulatedCoralPosition != null && simulatedCoralPosition > 0.1 && simulatedCoralPosition < 0.5));
        transportDIOSim.setValue(!(simulatedCoralPosition != null && simulatedCoralPosition > 0.7));

        if(simulatedCoralPosition != null) {
            double transportLength = Units.inchesToMeters(24);
            simulatedCoralPosition += transportSim.getAngularVelocityRadPerSec() / transportGearing
                * Units.inchesToMeters(2.9) * 1.3 / transportLength * 0.02;
            if(simulatedCoralPosition > 1) {
                simulatedCoralPosition = 1.;
            }
        }
        // TODO: Visualize simulated coral position

        if(intakeSimulation.obtainGamePieceFromIntake()) {
            simulatedCoralPosition = 0.;
        }

        Logger.recordOutput("Intake/SimulatedCoralPosition",
            simulatedCoralPosition == null ? -1 : simulatedCoralPosition);
        if(simulatedCoralPosition != null) SimRobotGamePiece.setCoralTransform(
            new Transform3d(simulatedCoralPosition * Units.inchesToMeters(28) - Units.inchesToMeters(12), 0.,
                Units.inchesToMeters(9), Rotation3d.kZero));

        super.updateInputs(inputs);

        if(Math.abs(inputs.intakePitch.getDegrees()) < MAX_INTAKE_ACTIVATION_PITCH
            && inputs.intakeWheelSpeed > MIN_INTAKE_WHEEL_SPEED &&
            // I don't currently want to deal with simulating issues with multiple game pieces, so we
            // just disallow intaking if we have a game piece.
            simulatedCoralPosition == null) {
            intakeSimulation.startIntake();
            Logger.recordOutput("Intake/SimulatedIntakeRunning", true);
        } else {
            intakeSimulation.stopIntake();
            Logger.recordOutput("Intake/SimulatedIntakeRunning", false);
        }
    }
}
