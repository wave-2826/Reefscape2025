package frc.robot.subsystems.pieceVision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

public class PieceVisionIOLimelight implements PieceVisionIO {
    /**
     * A subscriber for the pipeline latency of the Limelight. Must be added to the capture latency to get the total
     * latency. This is in milliseconds.
     */
    private final DoubleSubscriber pipelineLatencySubscriber;
    /**
     * A subscriber for the capture latency of the Limelight. This is the time between the end of the exposure of the
     * middle row of the sensor to the beginning of the tracking pipeline. This is in milliseconds.
     */
    private final DoubleSubscriber captureLatencySubscriber;
    /**
     * In the format of [id, txnc, tync, ta, corner0x, corner0y, corner1x, corner1y, corner2x, corner2y, corner3x,
     * corner3y, id2...] where id is the neural network class ID, txnc and tync are the target x and y in degrees from
     * the center of the camera, ta is the target area from 0-100, and corner0-3 are the corners of the target in image
     * space. I'm honestly not sure why they send all 4 corners instead of just 2 or a center and size. See
     * https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib#networktables-derived--rawdetection
     */
    private final DoubleArraySubscriber rawDetectionSubscriber;
    /**
     * A value that dictates how many frames will be skipped between processed frames. This reduces the temperature rise
     * of the Limelight, so we set it to 20 while the robot is disabled.
     */
    private final IntegerPublisher throttleSetPublisher;
    /**
     * A subscriber for hardware metrics of the Limelight in the format of [fps, cpu temp, ram usage, temp]. This
     * doesn't have a _super_ pracrical application other than being able to reflect on performance improvements or
     * differences between pipelines.
     */
    private final DoubleArraySubscriber hardwareMetricsSubscruber;

    /**
     * The timestamp of our last update. This is used to determine if we have a new observation.
     * @param hostname
     */
    private long lastUpdate = 0;

    public PieceVisionIOLimelight(String hostname) {
        var table = NetworkTableInstance.getDefault().getTable(hostname);
        pipelineLatencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
        captureLatencySubscriber = table.getDoubleTopic("cl").subscribe(0.0);
        rawDetectionSubscriber = table.getDoubleArrayTopic("rawdetections").subscribe(new double[0]);
        throttleSetPublisher = table.getIntegerTopic("throttle_set").publish();
        hardwareMetricsSubscruber = table.getDoubleArrayTopic("hw").subscribe(new double[0]);
    }

    @Override
    public void setEnabled(boolean enabled) {
        throttleSetPublisher.set(PieceVisionConstants.doNotReduceFramerateWhenDisabled ? 0
            : (enabled ? 0 : PieceVisionConstants.disabledThrottle));
    }

    @Override
    public void updateInputs(PieceVisionIOInputs inputs) {
        // Update connection status based on whether an update has been seen in the last 250ms
        inputs.connected = ((RobotController.getFPGATime() - pipelineLatencySubscriber.getLastChange()) / 1000) < 250;
        if(!inputs.connected) { return; }

        var hardwareMetrics = hardwareMetricsSubscruber.get();
        if(hardwareMetrics.length == 0) {
            inputs.framerate = 0;
            inputs.cpuTemp = 0;
        } else {
            inputs.framerate = hardwareMetrics[0];
            inputs.cpuTemp = hardwareMetrics[1];
        }

        var updateTime = rawDetectionSubscriber.getLastChange();
        if(updateTime == lastUpdate) {
            inputs.locations = null;
            return;
        }

        lastUpdate = updateTime;

        var rawDetectionWithTimestamp = rawDetectionSubscriber.getAtomic();
        var rawDetection = rawDetectionWithTimestamp.value;
        if(rawDetection.length == 0) {
            inputs.locations = null;
            return;
        }

        var locations = new PieceLocation[rawDetection.length / 12];
        for(int i = 0; i < locations.length; i++) {
            var offset = i * 12;
            // [id, txnc, tync, ta, corner0x, corner0y, corner1x, corner1y, corner2x, corner2y, corner3x, corner3y]
            // ID should always be 0 since we only detect one thing
            // Still, in case we ever switch to a more general system, discard any detections that aren't ID 0
            int id = (int) rawDetection[offset];
            if(id != 0) {
                locations[i] = null;
                continue;
            }

            var targetXDegrees = rawDetection[offset + 1];
            var targetYDegrees = rawDetection[offset + 2];
            var targetArea = rawDetection[offset + 3];

            // We currently don't care about the corners

            locations[i] = new PieceLocation(Rotation2d.fromDegrees(targetXDegrees),
                Rotation2d.fromDegrees(targetYDegrees), targetArea);
        }

        var timestamp = rawDetectionWithTimestamp.timestamp;
        var latency = captureLatencySubscriber.get() + pipelineLatencySubscriber.get();
        // Convert server timestamp from microseconds to seconds and adjust for latency
        double adjustedTimestampSeconds = (timestamp / 1000000.0) - (latency / 1000.0);

        inputs.locations = locations;
        inputs.timestamp = adjustedTimestampSeconds;
    }
}
