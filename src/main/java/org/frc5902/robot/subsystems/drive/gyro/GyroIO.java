package org.frc5902.robot.subsystems.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public GyroIOData data = new GyroIOData(false, new Rotation3d(), new Translation3d(), new Translation3d());
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    public default void resetGyro() {}

    // TREAT VELOCITY AND ACCELERATION AS FACTORS
    public record GyroIOData(
            boolean connected, Rotation3d position, Translation3d velocity, Translation3d acceleration) {}

    // public record GyroIOData(
    //         boolean connected,
    //         Rotation2d yawPosition,
    //         double yawVelocityRadPerSec,
    //         double yawAccelerationRadPerSec,
    //         Rotation2d pitchPosition,
    //         double pitchVelocityRadPerSec,
    //         double pitchAccelerationRadPerSec,
    //         Rotation2d rollPosition,
    //         double rollVelocityRadPerSec,
    //         double rollAccelerationRadPerSec) {}

    public default void resetGyro(Rotation3d pose) {}
}
