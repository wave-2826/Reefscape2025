package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    private final Alert driverDisconnectedAlert = new Alert("Driver controller disconnected (port 0)",
        AlertType.kWarning);
    private final Alert operatorDisconnectedAlert = new Alert("Operator controller disconnected (port 1)",
        AlertType.kWarning);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private final LoggedTunableNumber endgameAlert1Time = new LoggedTunableNumber("Controls/EndgameAlert1Time", 30.0);
    private final LoggedTunableNumber endgameAlert2Time = new LoggedTunableNumber("Controls/EndgameAlert2Time", 15.0);

    private static final Controls instance = new Controls();

    public static Controls getInstance() {
        return instance;
    }

    private Controls() {
        // This is a singleton
    }

    /** Configures the controls. */
    public void configureControls(Drive drive, SwerveDriveSimulation driveSimulation, Arm arm, Intake intake,
        Vision vision, Climber climber) {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(),
            () -> -driver.getRightX()));

        // Lock to 0° when A button is held
        driver.a().whileTrue(DriveCommands.joystickDriveAtAngle(drive, () -> -driver.getLeftY(),
            () -> -driver.getLeftX(), () -> Rotation2d.fromRadians(Math.atan2(driver.getRightY(), driver.getLeftY()))));

        // Switch to X pattern when X button is pressed
        driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // driver.b().whileTrue(AutoScoreCommands.au
        driver.b().toggleOnTrue(AutoScoreCommands.autoScoreStartCommand(drive, vision, arm, driver.rightBumper(),
            driver::getLeftX, driver::getLeftY));

        // Reset gyro or odometry if in simulation
        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
            ? () -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose()) // Reset odometry to actual robot pose during simulation
            : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)); // Zero gyro

        driver.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

        RobotModeTriggers.teleop().onTrue(ClimbCommands.resetClimbPosition());
        operator.start().whileTrue(ClimbCommands.climbCommand(climber, operator::getRightY));

        Container<Double> height = new Container<Double>(0.525);
        Container<Double> pitch = new Container<Double>(0.0);
        Container<Boolean> horizontal = new Container<Boolean>(false);
        operator.povLeft().onTrue(Commands.runOnce(() -> horizontal.value = !horizontal.value));
        operator.a().toggleOnTrue(arm.setTargetStateCommand(() -> {
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

        operator.x().whileTrue(IntakeCommands.intakeCommand(intake, operator::getLeftY, operator::getRightY));

        operator.b().onTrue(arm.goToStateCommand(ArmConstants.restingState));

        operator.rightBumper()
            .onTrue(arm.goToStateCommand(new ArmState(Rotation2d.fromDegrees(30), Inches.of(20),
                WristRotation.Horizontal, EndEffectorState.velocity(-6))))
            .onFalse(arm.goToStateCommand(ArmConstants.restingState));

        // operator.back().or(DriverStation::isTest).whileTrue(overrideControlsCommand(arm));
        operator.back().onTrue(Commands.runOnce(arm::resetToAbsolute));

        // Example Coral Placement Code
        // TODO: Implement this for our actual robot logic
        if(Constants.currentMode == Constants.Mode.SIM) {
            // L4 placement
            operator.y()
                .onTrue(Commands.runOnce(() -> SimulatedArena.getInstance()
                    .addGamePieceProjectile(new ReefscapeCoralOnFly(
                        driveSimulation.getSimulatedDriveTrainPose().getTranslation(), new Translation2d(0.4, 0),
                        driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        driveSimulation.getSimulatedDriveTrainPose().getRotation(), Meters.of(2),
                        MetersPerSecond.of(1.5), Degrees.of(-80)))));
            // L3 placement
            operator.b()
                .onTrue(Commands.runOnce(() -> SimulatedArena.getInstance()
                    .addGamePieceProjectile(new ReefscapeCoralOnFly(
                        driveSimulation.getSimulatedDriveTrainPose().getTranslation(), new Translation2d(0.4, 0),
                        driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        driveSimulation.getSimulatedDriveTrainPose().getRotation(), Meters.of(1.35),
                        MetersPerSecond.of(1.5), Degrees.of(-60)))));
        }

        // Endgame Alerts
        Trigger endgameAlert1Trigger = new Trigger(() -> DriverStation.isTeleopEnabled()
            && DriverStation.getMatchTime() > 0 && DriverStation.getMatchTime() <= endgameAlert1Time.get());
        Trigger endgameAlert2Trigger = new Trigger(() -> DriverStation.isTeleopEnabled()
            && DriverStation.getMatchTime() > 0 && DriverStation.getMatchTime() <= endgameAlert2Time.get());

        endgameAlert1Trigger.onTrue(controllerRumbleWhileRunning(true, false, RumbleType.kLeftRumble).withTimeout(0.5));
        endgameAlert2Trigger.onTrue(controllerRumbleWhileRunning(true, false, RumbleType.kLeftRumble).withTimeout(0.2)
            .andThen(Commands.waitSeconds(0.1)).repeatedly().withTimeout(0.9)); // Rumble three times
    }

    private Command overrideControlsCommand(Arm arm) {
        return Commands.run(() -> {
            // TODO
        }, arm);
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
