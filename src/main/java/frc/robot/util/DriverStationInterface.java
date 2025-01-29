package frc.robot.util;

import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.sun.net.httpserver.HttpServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants.ReefBranch;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.subsystems.drive.DriveConstants;

public class DriverStationInterface {
    /**
     * The port used for the driver station interface API. The FMS firewall allows traffic for team use on ports
     * 5800-5810. 5810 is already used by NetworkTables.
     */
    private static final int DS_API_PORT = 5809;

    private static DriverStationInterface instance;

    public static DriverStationInterface getInstance() {
        if(instance == null) instance = new DriverStationInterface();
        return instance;
    }

    /**
     * The selected reef branch entry in the NetworkTables table.
     */
    private LoggedNetworkString reefBranchEntry = new LoggedNetworkString("/DriverStationInterface/ReefBranch", "A");
    /**
     * The selected reef level entry in the NetworkTables table.
     */
    private LoggedNetworkString reefLevelEntry = new LoggedNetworkString("/DriverStationInterface/ReefLevel", "L1");

    /**
     * The HTTP server used for the driver station interface API.
     */
    private HttpServer server;

    private DriverStationInterface() {
        // Start the API server
        try {
            server = HttpServer.create(new InetSocketAddress(DS_API_PORT), 0);
            server.createContext("/autoData", (exchange) -> {
                String response = getAutoData();
                // Set CORS headers so we can access this from the frontend of the dashboard
                exchange.getResponseHeaders().add("Access-Control-Allow-Origin", "*");
                exchange.getResponseHeaders().add("Access-Control-Allow-Methods", "GET, OPTIONS");

                exchange.sendResponseHeaders(200, response.length());

                try(OutputStream os = exchange.getResponseBody()) {
                    os.write(response.getBytes());
                    os.close();
                }
            });
            server.start();
        } catch(IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Gets the autonomous dashboard data. Returns a JSON string.
     * @return
     */
    private String getAutoData() {
        StringBuilder jsonData = new StringBuilder();

        try {
            jsonData.append("{\"autoChoices\": [");

            List<String> autoNames = AutoBuilder.getAllAutoNames();
            for(String choice : autoNames) {
                jsonData.append("{\"name\": \"").append(choice).append("\", \"poses\": [");

                List<PathPlannerPath> paths = new ArrayList<>();
                paths = PathPlannerAuto.getPathGroupFromAutoFile(choice);
                for(PathPlannerPath path : paths) {
                    path.preventFlipping = true;
                }

                PathPlannerTrajectory simulatedPath = simulateAuto(paths, DriveConstants.pathplannerConfig);

                double sampleInterval = 0.02;
                double totalTime = simulatedPath.getTotalTimeSeconds();

                for(double t = 0; t < totalTime; t += sampleInterval) {
                    PathPlannerTrajectoryState state = simulatedPath.sample(t);

                    // @formatter:off
                    jsonData.append("{\"x\": ").append(Math.round(state.pose.getX() * 1000) / 1000.0)
                        .append(", \"y\": ").append(Math.round(state.pose.getY() * 1000) / 1000.0)
                        .append(", \"rot\": ").append(Math.round(state.pose.getRotation().getRadians() * 1000) / 1000.0)
                        .append(", \"t\": ").append(Math.round(t * 1000) / 1000.0)
                        .append("},");
                    // @formatter:on
                }

                if(totalTime > 0) jsonData.deleteCharAt(jsonData.length() - 1);
                jsonData.append("]},");
            }

            if(!autoNames.isEmpty()) jsonData.deleteCharAt(jsonData.length() - 1);
            jsonData.append("]}");
        } catch(Exception e) {
            System.out.println("Error loading auto data: " + e.getMessage());
        }

        return jsonData.toString();
    }

    static PathPlannerTrajectory simulateAuto(List<PathPlannerPath> paths, RobotConfig robotConfig) {
        if(paths.isEmpty()) return null;

        ArrayList<PathPlannerTrajectoryState> allStates = new ArrayList<>();

        PathPlannerPath firstPath = paths.get(0);
        Pose2d startPose = new Pose2d(firstPath.getAllPathPoints().get(0).position,
            firstPath.getIdealStartingState().rotation());
        ChassisSpeeds startSpeeds = new ChassisSpeeds();

        for(PathPlannerPath p : paths) {
            PathPlannerTrajectory simPath = new PathPlannerTrajectory(p, startSpeeds, startPose.getRotation(),
                robotConfig);

            PathPlannerTrajectoryState lastState = allStates.isEmpty() ? null : allStates.get(allStates.size() - 1);

            double startTime = lastState != null ? lastState.timeSeconds : 0;
            for(PathPlannerTrajectoryState s : simPath.getStates()) {
                s.timeSeconds += startTime;
                allStates.add(s);
            }

            lastState = allStates.get(allStates.size() - 1);
            startPose = new Pose2d(lastState.pose.getTranslation(), lastState.pose.getRotation());
            startSpeeds = lastState.fieldSpeeds;
        }

        return new PathPlannerTrajectory(allStates);
    }

    /**
     * Gets the reef target from the dashboard.
     * @return
     */
    @AutoLogOutput(key = "/DriverStationInterface/ReefTarget")
    public ReefTarget getReefTarget() {
        try {
            return new ReefTarget(ReefBranch.valueOf(reefBranchEntry.get()), ReefLevel.valueOf(reefLevelEntry.get()));
        } catch(IllegalArgumentException e) {
            DriverStation.reportError("Invalid reef target from dashboard", false);
            return new ReefTarget(ReefBranch.A, ReefLevel.L1);
        }
    }

    /**
     * Sets the reef target on the dashboard.
     * @param target
     */
    public void setReefTarget(ReefTarget target) {
        reefBranchEntry.set(target.branch().name());
        reefLevelEntry.set(target.level().name());
    }
}
