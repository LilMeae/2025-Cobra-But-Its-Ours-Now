package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {

    private final GyroSimulation gyroSim;

    public GyroIOSim(GyroSimulation gyroSim) {
        this.gyroSim = gyroSim;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = gyroSim.getGyroReading();
        inputs.yawVelocityRadPerSec = gyroSim.getMeasuredAngularVelocity().in(RadiansPerSecond);
        inputs.odometryYawTimestamps = new double[] {};
        inputs.odometryYawPositions = gyroSim.getCachedGyroReadings();
    }
}
