package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveTuningCommands;
import frc.robot.commands.vision.VisionTuningCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.EndEffectorState;
import frc.robot.subsystems.arm.ArmState.WristRotation;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.leds.LEDIO;
import frc.robot.subsystems.leds.LEDIORio;
import frc.robot.subsystems.leds.LEDIOSim;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.pieceVision.PieceVision;
import frc.robot.subsystems.pieceVision.PieceVisionIO;
import frc.robot.subsystems.pieceVision.PieceVisionIOLimelight;
import frc.robot.subsystems.pieceVision.PieceVisionIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.DriverStationInterface;

import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private final Vision vision;
    private final Arm arm;
    private final Climber climber;
    private final Intake intake;
    private final PieceVision pieceVision;

    @SuppressWarnings("unused")
    private final LEDs leds;

    // Only used in simulation
    private SwerveDriveSimulation driveSimulation = null;

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
                pieceVision = new PieceVision(new PieceVisionIOLimelight("limelight"), drive::getChassisSpeeds,
                    drive::getPose);
                arm = new Arm(new ArmIOReal());
                climber = new Climber(new ClimberIOReal());
                intake = new Intake(new IntakeIOReal());

                leds = new LEDs(new LEDIORio());
                break;
            case SIM:
                // Create a maple-sim swerve drive simulation instance
                this.driveSimulation = new SwerveDriveSimulation(DriveConstants.mapleSimConfig,
                    new Pose2d(2, 2, Rotation2d.kZero));
                // Add the simulated drivetrain to the simulation field
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(
                    new GyroIOSim(driveSimulation.getGyroSimulation(),
                        driveSimulation::getDriveTrainSimulatedChassisSpeedsRobotRelative),
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
                pieceVision = new PieceVision(new PieceVisionIOSim(), drive::getChassisSpeeds, drive::getPose);
                arm = new Arm(new ArmIOSim());
                climber = new Climber(new ClimberIOSim());
                intake = new Intake(new IntakeIOSim(driveSimulation));

                leds = new LEDs(new LEDIOSim());
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
                pieceVision = new PieceVision(new PieceVisionIO() {
                    /** Replayed robot doesn't have IO */
                }, drive::getChassisSpeeds, drive::getPose);
                arm = new Arm(new ArmIO() {
                    /** Replayed robot doesn't have IO */
                });
                climber = new Climber(new ClimberIO() {
                    /** Replayed robot doesn't have IO */
                });
                intake = new Intake(new IntakeIO() {
                    /** Replayed robot doesn't have IO */
                });

                leds = new LEDs(new LEDIO() {
                    /** Replayed robot doesn't have IO */
                });
                break;
        }

        // Initializes the driver station interface API.
        DriverStationInterface.getInstance();

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        DriveTuningCommands.addTuningCommandsToAutoChooser(drive, autoChooser);
        VisionTuningCommands.addTuningCommandsToAutoChooser(vision, autoChooser);

        // Configure the button bindings
        Controls.getInstance().configureControls(drive, driveSimulation, arm, intake, vision, climber);

        configureAutoCommands();
    }

    private void configureAutoCommands() {
        NamedCommands.registerCommand("L4", new SequentialCommandGroup(
            // new CloseLineupCommand(drive, null)
            arm.goToStateCommand(
                new ArmState(Rotation2d.fromDegrees(60), FieldConstants.ReefLevel.L4.height.minus(Meters.of(0.2)),
                    WristRotation.Vertical, EndEffectorState.hold())), // Line up
            arm.goToStateCommand(
                new ArmState(Rotation2d.fromDegrees(60), FieldConstants.ReefLevel.L4.height.minus(Meters.of(0.5)),
                    WristRotation.Vertical, EndEffectorState.velocity(2.0))) // Score
        ));
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void resetSimulationField() {
        if(Constants.currentMode != Constants.Mode.SIM) return;

        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void resetSimulatedRobot() {
        if(Constants.currentMode != Constants.Mode.SIM) return;

        driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, Rotation2d.kZero));
    }

    public void updateSimulation() {
        if(Constants.currentMode != Constants.Mode.SIM) return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput("FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput("FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));

        if(!VisionConstants.enableVisionSimulation) {
            drive.addVisionMeasurement(driveSimulation.getSimulatedDriveTrainPose(), Timer.getTimestamp(),
                VecBuilder.fill(0, 0, 0));
        }
    }
}
