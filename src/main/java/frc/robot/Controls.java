package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import java.util.HashMap;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoScoreCommands;
import frc.robot.commands.arm.ScoringSequenceCommands;
import frc.robot.commands.climber.ClimbCommands;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.EndEffectorState;
import frc.robot.subsystems.arm.ArmState.WristRotation;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.Container;
import frc.robot.util.DriverStationInterface;
import frc.robot.util.LoggedTunableNumber;

public class Controls {
    private static final double debounceTime = Constants.currentMode == Constants.Mode.SIM ? 0.15 : 0;

    private final Alert driverDisconnectedAlert = new Alert("Driver controller disconnected (port 0)",
        AlertType.kWarning);
    private final Alert operatorDisconnectedAlert = new Alert("Operator controller disconnected (port 1)",
        AlertType.kWarning);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private final LoggedTunableNumber endgameAlert1Time = new LoggedTunableNumber("Controls/EndgameAlert1Time", 30.0);
    private final LoggedTunableNumber endgameAlert2Time = new LoggedTunableNumber("Controls/EndgameAlert2Time", 15.0);

    private final Trigger normalOperator;
    private final Trigger operatorOverride;
    private final Trigger operatorManual;

    private static enum OperatorMode {
        Normal, Manual, Override
    };

    private OperatorMode operatorMode = OperatorMode.Normal;

    private static final Controls instance = new Controls();

    public static Controls getInstance() {
        return instance;
    }

    private Controls() {
        // This is a singleton
        normalOperator = new Trigger(() -> operatorMode == OperatorMode.Normal);
        operatorOverride = new Trigger(() -> operatorMode == OperatorMode.Override);
        operatorManual = new Trigger(() -> operatorMode == OperatorMode.Manual);
    }

    /** Configures the controls. */
    public void configureControls(Drive drive, SwerveDriveSimulation driveSimulation, Arm arm, Intake intake,
        Vision vision, Climber climber) {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(),
            () -> -driver.getRightX()));

        // Switch to X pattern when X button is pressed
        driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Auto score
        driver.b().debounce(Controls.debounceTime, DebounceType.kFalling)
            .whileTrue(AutoScoreCommands.autoScoreTeleopCommand(drive, vision, arm, driver.rightBumper(),
                driver::getLeftX, driver::getLeftY, (aligned) -> {
                    setDriverRumble(RumbleType.kLeftRumble, aligned ? 1.0 : 0.0, 1);
                }).finallyDo(() -> {
                    setDriverRumble(RumbleType.kLeftRumble, 0.0, 1);
                }));

        // Reset gyro or odometry if in simulation
        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
            ? () -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose()) // Reset odometry to actual robot pose during simulation
            : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)); // Zero gyro

        driver.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

        // Normal operator controls
        operator.y().and(normalOperator).whileTrue(ClimbCommands.climbCommand(climber, operator::getRightY));

        operator.rightBumper().and(normalOperator).whileTrue(IntakeCommands.intakeCommand(intake, arm));

        // Backup to stop the transport if the piece gets stuck somehow
        operator.povLeft().and(normalOperator).whileTrue(Commands.sequence(intake.setTransportOverrideSpeedCommand(0.0),
            arm.setTargetStateCommand(() -> ArmConstants.restingState)));

        // Backup for if the arm misses the piece somehow
        operator.leftBumper().and(normalOperator).onTrue(IntakeCommands.getPieceFromIntake(intake, arm));

        operator.b().and(normalOperator).onTrue(arm.goToStateCommand(ArmConstants.restingState));

        operator.povUp().and(normalOperator).onTrue(arm.goToStateCommand(ArmConstants.sourceIntakeState))
            .onFalse(arm.goToStateCommand(ArmConstants.sourceIntakeState.withEndEffector(EndEffectorState.hold())));

        operator.back().and(normalOperator).onTrue(Commands.runOnce(arm::resetToAbsolute));

        // Go to active scoring position
        operator.a().and(normalOperator).whileTrue(arm.goToStateCommand(() -> {
            return ScoringSequenceCommands
                .getStartingState(DriverStationInterface.getInstance().getReefTarget().level());
        }));

        // Override and manual mode enable
        operator.start().and(operator.back().negate()).debounce(0.05).onTrue(Commands.runOnce(() -> {
            if(operatorMode == OperatorMode.Manual) operatorMode = OperatorMode.Normal;
            else operatorMode = OperatorMode.Manual;
        }));
        operator.back().and(operator.start()).debounce(0.05).onTrue(Commands.runOnce(() -> {
            if(operatorMode == OperatorMode.Override) operatorMode = OperatorMode.Normal;
            else operatorMode = OperatorMode.Override;
        }));

        operatorManual.whileTrue(
            controllerRumbleWhileRunning(false, true, RumbleType.kRightRumble).withName("ManualOperatorControls"));
        operatorOverride.whileTrue(
            controllerRumbleWhileRunning(false, true, RumbleType.kLeftRumble).withName("OverrideOperatorControls"));

        // Manual operator controls
        Container<Double> height = new Container<Double>(0.525);
        Container<Double> pitch = new Container<Double>(0.0);
        Container<WristRotation> wristRotation = new Container<WristRotation>(WristRotation.Horizontal);
        operator.leftBumper().and(operatorManual).onTrue(Commands.runOnce(() -> {
            wristRotation.value = wristRotation.value.previous();
        }));
        operator.rightBumper().and(operatorManual).onTrue(Commands.runOnce(() -> {
            wristRotation.value = wristRotation.value.next();
        }));

        var intakeManualOverride = operator.b().and(operatorManual);
        intakeManualOverride.whileTrue(Commands.run(() -> {
            // TODO: SEVEN RIVERS - Change to closed loop control here
            intake.overridePitchPower(MathUtil.applyDeadband(operator.getRightY(), 0.2));
            intake.runIntakeOpenLoop(MathUtil.applyDeadband(operator.getLeftY(), 0.2) * 0.4);
        }, intake).withName("IntakeManualControls"));

        operatorManual.and(intakeManualOverride.negate()).whileTrue(arm.setTargetStateCommand(() -> {
            double eeSpeed = MathUtil.applyDeadband(operator.getLeftTriggerAxis() - operator.getRightTriggerAxis(), 0.2)
                * 100; // Rad/sec
            EndEffectorState endEffectorState = eeSpeed == 0.0 ? EndEffectorState.hold()
                : EndEffectorState.velocity(eeSpeed);

            height.value -= MathUtil.applyDeadband(operator.getLeftY(), 0.15) * 3.0 * 0.02;
            double minHeight = ArmConstants.ElevatorConstants.softStopMarginBottom.in(Meters);
            double maxHeight = ArmConstants.ElevatorConstants.maxElevatorHeight.in(Meters)
                - ArmConstants.ElevatorConstants.softStopMarginTop.in(Meters);
            height.value = MathUtil.clamp(height.value, minHeight, maxHeight);

            pitch.value -= MathUtil.applyDeadband(operator.getRightY(), 0.15) * 0.02 * 400.;
            pitch.value = MathUtil.clamp(pitch.value, ArmConstants.ShoulderConstants.minimumPitch.getDegrees(),
                ArmConstants.ShoulderConstants.maximumPitch.getDegrees());

            return new ArmState(Rotation2d.fromDegrees(pitch.value), Meters.of(height.value), wristRotation.value,
                endEffectorState);
        }).withName("ArmManualControls"));

        // Override operator controls
        var intakeOverride = operator.b().and(operatorOverride);
        var climberOverride = operator.y().and(operatorOverride);
        intakeOverride.whileTrue(Commands.run(() -> {
            intake.overridePitchPower(MathUtil.applyDeadband(operator.getRightY(), 0.2));
            intake.runIntakeOpenLoop(MathUtil.applyDeadband(operator.getLeftY(), 0.2) * 0.4);
        }, intake).withName("PitchOverrideControls"));
        climberOverride.whileTrue(Commands.run(() -> {
            climber.runClimberOpenLoop(MathUtil.applyDeadband(operator.getLeftY(), 0.2));
        }, climber).withName("ClimberOverrideControls"));

        operatorOverride.and(intakeOverride.negate()).and(climberOverride.negate()).whileTrue(Commands.run(() -> {
            // Arm control mode
            arm.overrideHeightPower(-MathUtil.applyDeadband(operator.getLeftY(), 0.2) * 0.3);
            arm.overridePitchPower(-MathUtil.applyDeadband(operator.getRightY(), 0.2) * 0.3);

            if(operator.leftBumper().getAsBoolean()) {
                arm.overrideWristPower(-0.25);
            } else if(operator.rightBumper().getAsBoolean()) {
                arm.overrideWristPower(0.25);
            } else {
                arm.overrideWristPower(0.0);
            }

            arm.overrideEndEffectorPower(
                MathUtil.applyDeadband(operator.getLeftTriggerAxis() - operator.getRightTriggerAxis(), 0.2) * 0.4);
        }, arm).withName("ArmOverrideControls"));

        // Simulation-specific controls
        if(Constants.currentMode == Constants.Mode.SIM) {
            // TODO
        }

        // Automatic mode actions
        RobotModeTriggers.teleop().onTrue(ClimbCommands.resetClimbPosition());

        // Endgame Alerts
        Trigger endgameAlert1Trigger = new Trigger(() -> DriverStation.isTeleopEnabled()
            && DriverStation.getMatchTime() > 0 && DriverStation.getMatchTime() <= endgameAlert1Time.get());
        Trigger endgameAlert2Trigger = new Trigger(() -> DriverStation.isTeleopEnabled()
            && DriverStation.getMatchTime() > 0 && DriverStation.getMatchTime() <= endgameAlert2Time.get());

        endgameAlert1Trigger.onTrue(controllerRumbleWhileRunning(true, false, RumbleType.kLeftRumble).withTimeout(0.5));
        endgameAlert2Trigger.onTrue(controllerRumbleWhileRunning(true, false, RumbleType.kLeftRumble).withTimeout(0.2)
            .andThen(Commands.waitSeconds(0.1)).repeatedly().withTimeout(0.9)); // Rumble three times
    }

    private HashMap<Integer, Double> driverRumbleCommands = new HashMap<>();
    private HashMap<Integer, Double> operatorRumbleCommands = new HashMap<>();

    public void setDriverRumble(RumbleType type, double value, int hash) {
        if(value == 0.0) {
            driverRumbleCommands.remove(hash);
        } else {
            driverRumbleCommands.put(hash, value);
        }
        driver.setRumble(type, driverRumbleCommands.values().stream().reduce(0.0, Double::max));
    }

    public void setOperatorRumble(RumbleType type, double value, int hash) {
        if(value == 0.0) {
            operatorRumbleCommands.remove(hash);
        } else {
            operatorRumbleCommands.put(hash, value);
        }
        operator.setRumble(type, operatorRumbleCommands.values().stream().reduce(0.0, Double::max));
    }

    public Command controllerRumbleWhileRunning(boolean forDriver, boolean forOperator, RumbleType type) {
        return Commands.startEnd(() -> {
            if(forDriver) setDriverRumble(type, 1.0, hashCode());
            if(forOperator) setOperatorRumble(type, 1.0, hashCode());
        }, () -> {
            if(forDriver) setDriverRumble(type, 0.0, hashCode());
            if(forOperator) setOperatorRumble(type, 0.0, hashCode());
        }).withName("ControllerRumbleWhileRunning");
    }

    /** Updates the controls, including shown alerts. */
    public void update() {
        // Controller disconnected alerts
        int driverPort = driver.getHID().getPort();
        int operatorPort = operator.getHID().getPort();
        driverDisconnectedAlert
            .set(!DriverStation.isJoystickConnected(driverPort) || !DriverStation.getJoystickIsXbox(driverPort));
        operatorDisconnectedAlert
            .set(!DriverStation.isJoystickConnected(operatorPort) || !DriverStation.getJoystickIsXbox(operatorPort));
    }
}
