package org.frc5902.robot.subsystems.launcher.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    class FlywheelIOInputs {
        public FlywheelIOData data = new FlywheelIOData(false, false, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    record FlywheelIOData(
            boolean leaderConnected,
            boolean followerConnected,
            double positionRotations,
            double velocityRPS,
            double appliedVoltage,
            double current,
            double tempCelsius) {}

    default void updateInputs(FlywheelIOInputs inputs) {}

    default void runVolts(double volts) {}

    default void runRotationsPerSecond(double rotationsPerSecond) {}

    default void stop() {}
}
