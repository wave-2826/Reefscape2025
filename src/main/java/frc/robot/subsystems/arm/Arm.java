package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

/**
 * The arm subsystem. Manages the elevator, arm pitch, arm wrist rotation, and end effector. We combine all these into
 * one subsystem because they need to closely interact; most of the time, the elevator, arm pitch, and arm wrist move
 * simultaneously, and the end effector must work with the arm wrist while rotating because we use a coaxial drive
 * system for it.
 */
public class Arm extends SubsystemBase {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs;

    private final ArmVisualizer visualizer = new ArmVisualizer("arm");

    private ArmState targetState;

    private final Alert elevatorMotorDisconnectedAlert = new Alert("Elevator motor disconnected!", AlertType.kError);
    private final Alert armPitchMotorDisconnectedAlert = new Alert("Arm pitch motor disconnected!", AlertType.kError);
    private final Alert armWristMotorDisconnectedAlert = new Alert("Arm wrist motor disconnected!", AlertType.kError);
    private final Alert endEffectorMotorDisconnectedAlert = new Alert("End effector motor disconnected!",
        AlertType.kError);

    // TODO: Make a utility class so we can merge multiple tunable PID values into one object
    private static final LoggedTunableNumber elevatorP = new LoggedTunableNumber("Arm/ElevatorP");
    private static final LoggedTunableNumber elevatorI = new LoggedTunableNumber("Arm/ElevatorI");
    private static final LoggedTunableNumber elevatorD = new LoggedTunableNumber("Arm/ElevatorD");

    private static final LoggedTunableNumber endEffectorP = new LoggedTunableNumber("Arm/EndEffectorP");
    private static final LoggedTunableNumber endEffectorI = new LoggedTunableNumber("Arm/EndEffectorI");
    private static final LoggedTunableNumber endEffectorD = new LoggedTunableNumber("Arm/EndEffectorD");
    private static final LoggedTunableNumber endEffectorF = new LoggedTunableNumber("Arm/EndEffectorF");

    private static final LoggedTunableNumber armPitchP = new LoggedTunableNumber("Arm/ArmPitchP");
    private static final LoggedTunableNumber armPitchI = new LoggedTunableNumber("Arm/ArmPitchI");
    private static final LoggedTunableNumber armPitchD = new LoggedTunableNumber("Arm/ArmPitchD");

    private static final LoggedTunableNumber armWristPositionP = new LoggedTunableNumber("Arm/WristPositionP");
    private static final LoggedTunableNumber armWristPositionI = new LoggedTunableNumber("Arm/WristPositionI");
    private static final LoggedTunableNumber armWristPositionD = new LoggedTunableNumber("Arm/WristPositionD");

    private static final LoggedTunableNumber armWristVelocityP = new LoggedTunableNumber("Arm/WristVelocityP");
    private static final LoggedTunableNumber armWristVelocityI = new LoggedTunableNumber("Arm/WristVelocityI");
    private static final LoggedTunableNumber armWristVelocityD = new LoggedTunableNumber("Arm/WristVelocityD");
    private static final LoggedTunableNumber armWristVelocityF = new LoggedTunableNumber("Arm/WristVelocityf");

    static {
        elevatorP.initDefault(ArmConstants.ElevatorConstants.elevatorKp);
        elevatorI.initDefault(ArmConstants.ElevatorConstants.elevatorKi);
        elevatorD.initDefault(ArmConstants.ElevatorConstants.elevatorKd);

        endEffectorP.initDefault(ArmConstants.EndEffectorConstants.endEffectorKp);
        endEffectorI.initDefault(ArmConstants.EndEffectorConstants.endEffectorKi);
        endEffectorD.initDefault(ArmConstants.EndEffectorConstants.endEffectorKd);
        endEffectorF.initDefault(ArmConstants.EndEffectorConstants.endEffectorKf);

        armPitchP.initDefault(ArmConstants.ShoulderConstants.armPitchKp);
        armPitchI.initDefault(ArmConstants.ShoulderConstants.armPitchKi);
        armPitchD.initDefault(ArmConstants.ShoulderConstants.armPitchKd);

        armWristPositionP.initDefault(ArmConstants.ShoulderConstants.armWristPositionKp);
        armWristPositionI.initDefault(ArmConstants.ShoulderConstants.armWristPositionKi);
        armWristPositionD.initDefault(ArmConstants.ShoulderConstants.armWristPositionKd);

        armWristVelocityP.initDefault(ArmConstants.ShoulderConstants.armWristVelocityKp);
        armWristVelocityI.initDefault(ArmConstants.ShoulderConstants.armWristVelocityKi);
        armWristVelocityD.initDefault(ArmConstants.ShoulderConstants.armWristVelocityKd);
        armWristVelocityF.initDefault(ArmConstants.ShoulderConstants.armWristVelocityKf);
    }

    public Arm(ArmIO io) {
        this.io = io;
        this.inputs = new ArmIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        if(elevatorP.hasChanged(hashCode()) || elevatorI.hasChanged(hashCode()) || elevatorD.hasChanged(hashCode())) {
            io.setElevatorGains(elevatorP.get(), elevatorI.get(), elevatorD.get());
        }
        if(endEffectorP.hasChanged(hashCode()) || endEffectorI.hasChanged(hashCode())
            || endEffectorD.hasChanged(hashCode()) || endEffectorF.hasChanged(hashCode())) {
            io.setEndEffectorGains(endEffectorP.get(), endEffectorI.get(), endEffectorD.get(), endEffectorF.get());
        }
        if(armPitchP.hasChanged(hashCode()) || armPitchI.hasChanged(hashCode()) || armPitchD.hasChanged(hashCode())) {
            io.setArmPitchGains(armPitchP.get(), armPitchI.get(), armPitchD.get());
        }
        if(armWristPositionP.hasChanged(hashCode()) || armWristPositionI.hasChanged(hashCode())
            || armWristPositionD.hasChanged(hashCode())) {
            io.setArmWristPositionGains(armWristPositionP.get(), armWristPositionI.get(), armWristPositionD.get());
        }
        if(armWristVelocityP.hasChanged(hashCode()) || armWristVelocityI.hasChanged(hashCode())
            || armWristVelocityD.hasChanged(hashCode()) || armWristVelocityF.hasChanged(hashCode())) {
            io.setArmWristVelocityGains(armWristVelocityP.get(), armWristVelocityI.get(), armWristVelocityD.get(),
                armWristVelocityF.get());
        }

        io.setElevatorHeight(0.5);
        io.setArmPitchPosition(Rotation2d.fromDegrees(45));

        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        visualizer.update(inputs.elevatorHeightMeters, inputs.armPitchPosition, inputs.armWristPosition,
            inputs.gamePiecePresent);

        // Update alerts
        elevatorMotorDisconnectedAlert.set(!inputs.elevatorMotorsConnected);
        armPitchMotorDisconnectedAlert.set(!inputs.armPitchMotorConnected);
        armWristMotorDisconnectedAlert.set(!inputs.armWristMotorConnected);
        endEffectorMotorDisconnectedAlert.set(!inputs.endEffectorMotorConnected);
    }
}
