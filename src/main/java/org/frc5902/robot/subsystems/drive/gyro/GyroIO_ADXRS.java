package org.frc5902.robot.subsystems.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import org.frc5902.robot.subsystems.drive.SparkOdometryThread;

import java.util.Queue;

public class GyroIO_ADXRS implements GyroIO {
    public final ADXRS450_Gyro ADXRS_Gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroIO_ADXRS() {
        ADXRS_Gyro.reset();
        yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(ADXRS_Gyro::getAngle);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.data = new GyroIOData(
                ADXRS_Gyro.isConnected(),
                Rotation2d.fromDegrees(-ADXRS_Gyro.getAngle()),
                -ADXRS_Gyro.getRate(),
                0,
                Rotation2d.kZero,
                0,
                0,
                Rotation2d.kZero,
                0,
                0);

        inputs.odometryYawTimestamps =
                yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromDegrees(-value))
                .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }

    public void resetGyroscope() {
        ADXRS_Gyro.reset();
    }
}
