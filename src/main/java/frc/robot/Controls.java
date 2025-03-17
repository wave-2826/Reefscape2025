package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

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
    private boolean isInOverrideMode = false;

    private static final Controls instance = new Controls();

    public static Controls getInstance() {
        return instance;
    }

    private Controls() {
        // This is a singleton
        normalOperator = new Trigger(() -> !isInOverrideMode);
        operatorOverride = new Trigger(() -> isInOverrideMode);
    }

    /** Configures the controls. */
    public void configureControls(Drive drive, SwerveDriveSimulation driveSimulation, Arm arm, Intake intake,
        Vision vision, Climber climber) {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(),
            () -> -driver.getRightX()));

        driver.a().whileTrue(DriveCommands.joystickDriveAtAngle(drive, () -> -driver.getLeftY(),
            () -> -driver.getLeftX(), () -> Rotation2d.fromRadians(Math.atan2(driver.getRightY(), driver.getLeftY()))));

        // Switch to X pattern when X button is pressed
        driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        driver.b().debounce(Controls.debounceTime, DebounceType.kFalling).whileTrue(AutoScoreCommands
            .autoScoreStartCommand(drive, vision, arm, driver.rightBumper(), driver::getLeftX, driver::getLeftY));

        // Reset gyro or odometry if in simulation
        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
            ? () -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose()) // Reset odometry to actual robot pose during simulation
            : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)); // Zero gyro

        driver.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

        // Normal operator controls
        operator.start().and(normalOperator).whileTrue(ClimbCommands.climbCommand(climber, operator::getRightY));

        // Temporary manual override mode
        Container<Double> height = new Container<Double>(0.525);
        Container<Double> pitch = new Container<Double>(0.0);
        Container<Boolean> horizontal = new Container<Boolean>(false);
        operator.povLeft().and(normalOperator).onTrue(Commands.runOnce(() -> horizontal.value = !horizontal.value));
        operator.a().and(normalOperator).toggleOnTrue(arm.setTargetStateCommand(() -> {
            boolean controllingHeight = operator.leftBumper().getAsBoolean();
            double eeSpeed = MathUtil.applyDeadband(controllingHeight ? 0. : operator.getLeftY(), 0.15) * -200.; // Rad/sec
            EndEffectorState endEffectorState = eeSpeed == 0.0 ? EndEffectorState.hold()
                : EndEffectorState.velocity(eeSpeed);
            double speed = 4.0;

            height.value -= controllingHeight ? (MathUtil.applyDeadband(operator.getLeftY(), 0.15) * speed * 0.02) : 0.;
            pitch.value -= MathUtil.applyDeadband(operator.getRightY(), 0.15) * 0.02 * 400.;
            return new ArmState(Rotation2d.fromDegrees(pitch.value), Meters.of(height.value),
                horizontal.value ? WristRotation.Horizontal : WristRotation.Vertical, endEffectorState);
        }));

        operator.rightBumper().and(normalOperator).whileTrue(IntakeCommands.intakeCommand(intake, arm));

        operator.b().and(normalOperator).onTrue(arm.goToStateCommand(ArmConstants.restingState));

        operator.leftBumper().and(normalOperator)
            .onTrue(arm.goToStateCommand(new ArmState(Rotation2d.fromDegrees(30), Inches.of(20),
                WristRotation.Horizontal, EndEffectorState.velocity(-6))))
            .onFalse(arm.goToStateCommand(ArmConstants.restingState));

        operator.back().and(normalOperator).onTrue(Commands.runOnce(arm::resetToAbsolute));

        // Override mode enable
        operator.back().and(operator.start()).toggleOnTrue(Commands.startEnd(() -> {
            isInOverrideMode = true;
        }, () -> {
            isInOverrideMode = false;
        }).alongWith(controllerRumbleWhileRunning(false, true, RumbleType.kLeftRumble)));

        // Override operator controls
        var intakeOverride = operator.b().and(operatorOverride);
        var climberOverride = operator.y().and(operatorOverride);
        intakeOverride.whileTrue(Commands.run(() -> {
            intake.overridePitchPower(MathUtil.applyDeadband(operator.getRightY(), 0.2));
            intake.runIntakeOpenLoop(MathUtil.applyDeadband(operator.getLeftY(), 0.2) * 0.4);
        }, intake));
        climberOverride.whileTrue(Commands.run(() -> {
            climber.runClimberOpenLoop(MathUtil.applyDeadband(operator.getLeftY(), 0.2));
        }, climber));

        normalOperator.or(intakeOverride).or(climberOverride).whileFalse(Commands.run(() -> {
            // Arm control mode
            arm.overrideHeightPower(-MathUtil.applyDeadband(operator.getLeftY(), 0.2) * 0.3);
            arm.overridePitchPower(-MathUtil.applyDeadband(operator.getRightY(), 0.2) * 0.3);

            if(operator.leftBumper().getAsBoolean()) {
                arm.overrideWristPower(-0.1);
            } else if(operator.rightBumper().getAsBoolean()) {
                arm.overrideWristPower(0.1);
            } else {
                arm.overrideWristPower(0.0);
            }

            arm.overrideEndEffectorPower(
                MathUtil.applyDeadband(operator.getLeftTriggerAxis() - operator.getRightTriggerAxis(), 0.2) * 0.4);
        }, arm));

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

    private double operatorOverrideRumbleLeft = 0.0;
    private double operatorOverrideRumbleRight = 0.0;
    private double driverOverrideRumbleLeft = 0.0;
    private double driverOverrideRumbleRight = 0.0;

    private void setOperatorRumble(RumbleType type, double value) {
        if(type == RumbleType.kBothRumble || type == RumbleType.kLeftRumble) operator.setRumble(RumbleType.kLeftRumble,
            Math.max(operatorOverrideRumbleLeft, value));
        if(type == RumbleType.kBothRumble || type == RumbleType.kRightRumble) operator
            .setRumble(RumbleType.kRightRumble, Math.max(operatorOverrideRumbleRight, value));
    }

    private void setDriverRumble(RumbleType type, double value) {
        if(type == RumbleType.kBothRumble || type == RumbleType.kLeftRumble) driver.setRumble(RumbleType.kLeftRumble,
            Math.max(driverOverrideRumbleLeft, value));
        if(type == RumbleType.kBothRumble || type == RumbleType.kRightRumble) driver.setRumble(RumbleType.kRightRumble,
            Math.max(driverOverrideRumbleRight, value));
    }

    public void setOperatorOverrideRumble(double left, double right) {
        operatorOverrideRumbleLeft = left;
        operatorOverrideRumbleRight = right;
    }

    public void setDriverOverrideRumble(double left, double right) {
        driverOverrideRumbleLeft = left;
        driverOverrideRumbleRight = right;
    }

    public Command controllerRumbleWhileRunning(boolean forDriver, boolean forOperator, RumbleType type) {
        return Commands.startEnd(() -> {
            if(forDriver) setDriverRumble(type, 1.0);
            if(forOperator) setOperatorRumble(type, 1.0);
        }, () -> {
            if(forDriver) setDriverRumble(type, 0.0);
            if(forOperator) setOperatorRumble(type, 0.0);
        });
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
