package frc.robot.commands.vision;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.vision.Vision;

public class VisionTuningCommands {
    private static class TransformAverage {
        private double xSum = 0;
        private double ySum = 0;
        private double zSum = 0;
        private double pitchSum = 0;
        private double rollSum = 0;
        private double yawSum = 0;
        private int datapoints = 0;

        public TransformAverage() {
        }

        public void add(Transform3d transform) {
            xSum += transform.getX();
            ySum += transform.getY();
            zSum += transform.getZ();
            pitchSum += transform.getRotation().getY();
            rollSum += transform.getRotation().getX();
            yawSum += transform.getRotation().getZ();
            datapoints += 1;
        }

        public Transform3d getAverage() {
            if(datapoints == 0) return new Transform3d();
            return new Transform3d(new Translation3d(xSum / datapoints, ySum / datapoints, zSum / datapoints),
                new Rotation3d(rollSum / datapoints, pitchSum / datapoints, yawSum / datapoints));
        }
    }

    /** Adds the drive tuning commands to the auto chooser. */
    public static void addTuningCommandsToAutoChooser(Vision vision, LoggedDashboardChooser<Command> chooser) {
        // We may want to run this at a competition
        chooser.addOption("TUNING | Vision Camera Position Measurement", measureCameraPositions(vision));
    }

    /** The transform of the calibration tag, relative to the robot base. */
    public static Transform3d heldTagTransform = new Transform3d(new Translation3d( //
        Units.inchesToMeters(15. + 6.5), // Distance forward
        Units.inchesToMeters((2.379 - 3.004) / 2), // Distance left
        Units.inchesToMeters(8.45) // Distance up
    ), new Rotation3d(0., 0., Units.degreesToRadians(180)));

    public static Command measureCameraPositions(Vision vision) {
        TransformAverage[] averages = new TransformAverage[vision.getCameraCount()];
        return Commands.startRun(() -> {
            for(int cameraIndex = 0; cameraIndex < vision.getCameraCount(); cameraIndex++) {
                averages[cameraIndex] = new TransformAverage();
            }
            System.out.println("********** Vision camera position measurement started. **********");
        }, () -> {

            Transform3d[] transforms = vision.getBestTagTransforms();
            for(int cameraIndex = 0; cameraIndex < vision.getCameraCount(); cameraIndex++) {
                averages[cameraIndex].add(transforms[cameraIndex]);
            }
        }, vision).finallyDo(() -> {
            System.out.println("********** Vision camera position measurement results **********");
            for(int cameraIndex = 0; cameraIndex < vision.getCameraCount(); cameraIndex++) {
                Transform3d averageTransform = averages[cameraIndex].getAverage();
                Transform3d transform = heldTagTransform.plus(averageTransform.inverse());
                Rotation3d rotation = transform.getRotation();

                // new Transform3d(new Translation3d(x, y, z), new Rotation3d(roll, pitch, yaw))
                System.out.println("Robot to camera " + cameraIndex + ": `new Transform3d(new Translation3d("
                    + transform.getX() + ", " + transform.getY() + ", " + transform.getZ() + "), new Rotation3d("
                    + rotation.getX() + ", " + rotation.getY() + ", " + rotation.getZ() + "))`");
            }
        });
    }
}
