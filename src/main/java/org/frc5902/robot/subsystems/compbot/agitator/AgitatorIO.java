package org.frc5902.robot.subsystems.compbot.agitator;

import org.littletonrobotics.junction.AutoLog;

public interface AgitatorIO {
    @AutoLog
    class AgitatorIOInputs {
        public AgitatorIOData data = new AgitatorIOData(false, 0.0, 0.0, 0.0, 0.0);
    }

    record AgitatorIOData(
            boolean motorConnected,
            double positionRads,
            double velocityRadsPerSec,
            double appliedVoltage,
            double tempCelsius) {}

    default void updateInputs(AgitatorIOInputs inputs) {}

    default void runVolts(double volts) {}

    default void runRadiansPerSecond(double radiansPerSecond) {}

    default void stop() {}
}
