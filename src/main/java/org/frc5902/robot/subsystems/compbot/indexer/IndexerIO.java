package org.frc5902.robot.subsystems.compbot.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    class IndexerIOInputs {
        public IndexerIOData data = new IndexerIOData(false, 0, 0, 0, 0, 0);
    }


    record IndexerIOData(
        boolean motorConnected,
        double positionRads,
        double velocityRadsPerSec,
        double appliedVoltage,
        double tempCelsius,
        double current
    ) {}

    default void updateInputs(IndexerIOInputs inputs) {}

    default void runVolts(double volts) {}

    default void runRadiansPerSecond(double radiansPerSecond) {}

    default void stop() {}
}
