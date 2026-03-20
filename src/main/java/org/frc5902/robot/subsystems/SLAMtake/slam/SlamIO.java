package org.frc5902.robot.subsystems.SLAMtake.slam;

import org.littletonrobotics.junction.AutoLog;

public interface SlamIO {
    @AutoLog
    class SlamIOInputs {
        public SlamIOData data = new SlamIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    record SlamIOData(
            boolean motorConnected,
            double positionRads,
            double velocityRadsPerSec,
            double appliedVoltage,
            double tempCelsius,
            double current) {}

    default void updateInputs(SlamIOInputs inputs) {}

    default void runVolts(double volts) {}

    default void runRadiansPerSecond(double radiansPerSecond) {}

    default void stop() {}
}
