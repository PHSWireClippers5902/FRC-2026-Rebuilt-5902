package org.frc5902.robot.subsystems.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import org.frc5902.robot.subsystems.drive.SparkOdometryThread;

import java.util.Queue;
import java.util.function.DoubleSupplier;

public class GyroIO_ADIS implements GyroIO {
    public final ADIS16470_IMU ADIS_Gyro;

    public final DoubleSupplier yaw;
    public final DoubleSupplier pitch;
    public final DoubleSupplier roll;

    public final DoubleSupplier yawVelocity;
    public final DoubleSupplier pitchVelocity;
    public final DoubleSupplier rollVelocity;

    public final DoubleSupplier yawAcceleration;
    public final DoubleSupplier pitchAcceleration;
    public final DoubleSupplier rollAcceleration;

    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroIO_ADIS() {
        ADIS_Gyro = new ADIS16470_IMU();
        ADIS_Gyro.calibrate();
        ADIS_Gyro.reset();
        ADIS_Gyro.setGyroAngle(IMUAxis.kYaw, 0.0);
        yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(ADIS_Gyro::getAngle);

        yaw = () -> ADIS_Gyro.getAngle(IMUAxis.kYaw);
        pitch = () -> ADIS_Gyro.getAngle(IMUAxis.kPitch);
        roll = () -> ADIS_Gyro.getAngle(IMUAxis.kRoll);

        yawVelocity = () -> ADIS_Gyro.getRate(IMUAxis.kYaw);
        pitchVelocity = () -> ADIS_Gyro.getRate(IMUAxis.kPitch);
        rollVelocity = () -> ADIS_Gyro.getRate(IMUAxis.kRoll);

        yawAcceleration = () -> ADIS_Gyro.getAccelZ();
        pitchAcceleration = () -> ADIS_Gyro.getAccelX();
        rollAcceleration = () -> ADIS_Gyro.getAccelY();
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

    @Override
    public void resetGyro() {
        ADIS_Gyro.reset();
    }
}
