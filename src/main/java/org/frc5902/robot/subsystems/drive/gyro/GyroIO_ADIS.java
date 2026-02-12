package org.frc5902.robot.subsystems.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import org.frc5902.robot.subsystems.drive.SparkOdometryThread;

import java.util.Queue;
import java.util.function.DoubleSupplier;

public class GyroIO_ADIS implements GyroIO {
    public final ADIS16470_IMU ADIS_Gyro = new ADIS16470_IMU();

    public DoubleSupplier yaw = () -> ADIS_Gyro.getAngle(IMUAxis.kYaw);
    public DoubleSupplier pitch = () -> ADIS_Gyro.getAngle(IMUAxis.kPitch);
    public DoubleSupplier roll = () -> ADIS_Gyro.getAngle(IMUAxis.kRoll);

    public DoubleSupplier yawVelocity = () -> ADIS_Gyro.getRate(IMUAxis.kYaw);
    public DoubleSupplier pitchVelocity = () -> ADIS_Gyro.getRate(IMUAxis.kPitch);
    public DoubleSupplier rollVelocity = () -> ADIS_Gyro.getRate(IMUAxis.kRoll);

    public DoubleSupplier yawAcceleration = () -> ADIS_Gyro.getAccelZ();
    public DoubleSupplier pitchAcceleration = () -> ADIS_Gyro.getAccelX();
    public DoubleSupplier rollAcceleration = () -> ADIS_Gyro.getAccelY();

    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroIO_ADIS() {
        ADIS_Gyro.calibrate();
        ADIS_Gyro.setGyroAngle(IMUAxis.kYaw, 0.0);
        yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(ADIS_Gyro::getAngle);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.data = new GyroIOData(
                ADIS_Gyro.isConnected(),
                Rotation2d.fromDegrees(yaw.getAsDouble()),
                Units.degreesToRadians(yawVelocity.getAsDouble()),
                yawAcceleration.getAsDouble(),
                Rotation2d.fromDegrees(pitch.getAsDouble()),
                Units.degreesToRadians(pitchVelocity.getAsDouble()),
                pitchAcceleration.getAsDouble(),
                Rotation2d.fromDegrees(roll.getAsDouble()),
                Units.degreesToRadians(rollVelocity.getAsDouble()),
                rollAcceleration.getAsDouble());
        inputs.odometryYawTimestamps =
                yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromDegrees(value))
                .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
