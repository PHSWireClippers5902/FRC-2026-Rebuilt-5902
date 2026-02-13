package org.frc5902.robot.subsystems.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public GyroIOData data =
                new GyroIOData(false, Rotation2d.kZero, 0, 0, Rotation2d.kZero, 0, 0, Rotation2d.kZero, 0, 0);
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    public default void resetGyro() {}

    public record GyroIOData(
            boolean connected,
            Rotation2d yawPosition,
            double yawVelocityRadPerSec,
            double yawAccelerationRadPerSec,
            Rotation2d pitchPosition,
            double pitchVelocityRadPerSec,
            double pitchAccelerationRadPerSec,
            Rotation2d rollPosition,
            double rollVelocityRadPerSec,
            double rollAccelerationRadPerSec) {}
}
