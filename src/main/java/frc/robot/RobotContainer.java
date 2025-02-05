package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.DriverStationInterface;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    // Subsystems
    private final Drive drive;
    @SuppressWarnings("unused")
    private final Vision vision;
    @SuppressWarnings("unused")
    private final Arm arm;

    // Only used in simulation
    private SwerveDriveSimulation driveSimulation = null;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch(Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(new GyroIOPigeon2(), new ModuleIOSpark(DriveConstants.frontLeftModule),
                    new ModuleIOSpark(DriveConstants.frontRightModule),
                    new ModuleIOSpark(DriveConstants.backLeftModule), new ModuleIOSpark(DriveConstants.backRightModule),
                    (pose) -> {
                    });
                vision = new Vision(drive::addVisionMeasurement,
                    new VisionIOPhotonVision(VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                    new VisionIOPhotonVision(VisionConstants.camera1Name, VisionConstants.robotToCamera1),
                    new VisionIOPhotonVision(VisionConstants.camera2Name, VisionConstants.robotToCamera2),
                    new VisionIOPhotonVision(VisionConstants.camera3Name, VisionConstants.robotToCamera3));
                arm = new Arm(new ArmIOReal());
                break;
            case SIM:
                // Create a maple-sim swerve drive simulation instance
                this.driveSimulation = new SwerveDriveSimulation(DriveConstants.mapleSimConfig,
                    new Pose2d(2, 2, new Rotation2d()));
                // Add the simulated drivetrain to the simulation field
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(new GyroIOSim(driveSimulation.getGyroSimulation()),
                    new ModuleIOSim(driveSimulation.getModules()[0]), new ModuleIOSim(driveSimulation.getModules()[1]),
                    new ModuleIOSim(driveSimulation.getModules()[2]), new ModuleIOSim(driveSimulation.getModules()[3]),
                    driveSimulation::setSimulationWorldPose);

                vision = new Vision(drive::addVisionMeasurement,
                    new VisionIOPhotonVisionSim(VisionConstants.camera0Name, VisionConstants.robotToCamera0,
                        driveSimulation::getSimulatedDriveTrainPose),
                    new VisionIOPhotonVisionSim(VisionConstants.camera1Name, VisionConstants.robotToCamera1,
                        driveSimulation::getSimulatedDriveTrainPose),
                    new VisionIOPhotonVisionSim(VisionConstants.camera2Name, VisionConstants.robotToCamera2,
                        driveSimulation::getSimulatedDriveTrainPose),
                    new VisionIOPhotonVisionSim(VisionConstants.camera3Name, VisionConstants.robotToCamera3,
                        driveSimulation::getSimulatedDriveTrainPose));
                arm = new Arm(new ArmIOSim());
                break;
            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(new GyroIO() {
                    /** Replayed robot doesn't have IO */
                }, new ModuleIO() {
                    /** Replayed robot doesn't have IO */
                }, new ModuleIO() {
                    /** Replayed robot doesn't have IO */
                }, new ModuleIO() {
                    /** Replayed robot doesn't have IO */
                }, new ModuleIO() {
                    /** Replayed robot doesn't have IO */
                }, (pose) -> {
                });
                // Needs to use the same number of dummy implementations as the real robot has cameras
                vision = new Vision(drive::addVisionMeasurement, new VisionIO() {
                    /** Replayed robot doesn't have IO */
                }, new VisionIO() {
                    /** Replayed robot doesn't have IO */
                }, new VisionIO() {
                    /** Replayed robot doesn't have IO */
                }, new VisionIO() {
                    /** Replayed robot doesn't have IO */
                });
                arm = new Arm(new ArmIO() {
                    /** Replayed robot doesn't have IO */
                });
                break;
        }

        // Initializes the driver station interface API.
        DriverStationInterface.getInstance();

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption("Drive SysId (Quasistatic Forward)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Quasistatic Reverse)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        configureButtonBindings();
    }

    /** Defines button to command mappings. */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(drive, () -> -controller.getLeftY(),
            () -> -controller.getLeftX(), () -> -controller.getRightX()));

        // Lock to 0° when A button is held
        controller.a().whileTrue(
            DriveCommands.joystickDriveAtAngle(drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(),
                () -> Rotation2d.fromRadians(Math.atan2(controller.getRightY(), controller.getLeftY()))));

        // Switch to X pattern when X button is pressed
        controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro or odometry if in simulation
        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
            ? () -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose()) // Reset odometry to actual robot pose during simulation
            : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // Zero gyro

        controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

        // Example Coral Placement Code
        // TODO: Implement this for our actual robot logic
        if(Constants.currentMode == Constants.Mode.SIM) {
            // L4 placement
            controller.y()
                .onTrue(Commands.runOnce(() -> SimulatedArena.getInstance()
                    .addGamePieceProjectile(new ReefscapeCoralOnFly(
                        driveSimulation.getSimulatedDriveTrainPose().getTranslation(), new Translation2d(0.4, 0),
                        driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        driveSimulation.getSimulatedDriveTrainPose().getRotation(), Meters.of(2),
                        MetersPerSecond.of(1.5), Degrees.of(-80)))));
            // L3 placement
            controller.b()
                .onTrue(Commands.runOnce(() -> SimulatedArena.getInstance()
                    .addGamePieceProjectile(new ReefscapeCoralOnFly(
                        driveSimulation.getSimulatedDriveTrainPose().getTranslation(), new Translation2d(0.4, 0),
                        driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        driveSimulation.getSimulatedDriveTrainPose().getRotation(), Meters.of(1.35),
                        MetersPerSecond.of(1.5), Degrees.of(-60)))));
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void resetSimulationField() {
        if(Constants.currentMode != Constants.Mode.SIM) return;

        driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if(Constants.currentMode != Constants.Mode.SIM) return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput("FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput("FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }
}
