package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SparkUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {
    private final GyroSimulation gyroSimulation;

    private final Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier;
    private ChassisSpeeds lastChassisSpeeds = new ChassisSpeeds();

    public GyroIOSim(GyroSimulation gyroSimulation, Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier) {
        this.gyroSimulation = gyroSimulation;
        this.robotRelativeSpeedsSupplier = robotRelativeSpeedsSupplier;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = gyroSimulation.getGyroReading();
        inputs.yawVelocityRadPerSec = Units
            .degreesToRadians(gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond));

        var currentChassisSpeeds = robotRelativeSpeedsSupplier.get();
        var speedChange = currentChassisSpeeds.minus(lastChassisSpeeds);
        lastChassisSpeeds = currentChassisSpeeds;

        var acceleration = Math.sqrt(speedChange.vxMetersPerSecond * speedChange.vxMetersPerSecond
            + speedChange.vyMetersPerSecond * speedChange.vyMetersPerSecond) / 0.02;

        inputs.accelerationGs = acceleration / 9.81;

        inputs.odometryYawTimestamps = SparkUtil.getSimulationOdometryTimeStamps();
        inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
    }
}
