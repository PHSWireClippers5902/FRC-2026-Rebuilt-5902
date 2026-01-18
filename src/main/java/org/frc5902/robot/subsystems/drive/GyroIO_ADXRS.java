package org.frc5902.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;

import java.util.Queue;

import org.frc5902.robot.util.SparkOdometryThread;

public class GyroIO_ADXRS implements GyroIO {
    public final ADXRS450_Gyro ADXRS_Gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroIO_ADXRS() {
        yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(ADXRS_Gyro::getAngle);
    }
    // TODO CONFIRM THAT GYROSCOPE IS REVERSED....
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = ADXRS_Gyro.isConnected();
        inputs.yawPosition = Rotation2d.fromDegrees(-ADXRS_Gyro.getAngle());
        // no cool port :(
        inputs.yawVelocityRadiansPerSeconds = Units.degreesToRadians(-ADXRS_Gyro.getAngle());

        inputs.odometryYawTimestamps = 
            yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions = 
            yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromDegrees(-value))
                .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
