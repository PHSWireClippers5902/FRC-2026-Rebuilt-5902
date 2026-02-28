package org.frc5902.robot.subsystems.compbot.slider;

import org.littletonrobotics.junction.AutoLog;

public interface SliderIO {
    @AutoLog
    class SliderIOInputs {
        public SliderIOData data = new SliderIOData(false, 0.0, 0.0, 0.0, 0.0, false);
    }

    record SliderIOData(
            boolean motorConnected,
            double positionRads,
            double velocityRadsPerSec,
            double appliedVoltage,
            double tempCelsius,
            boolean limitSwitchActivated) {}

    default void updateInputs(SliderIOInputs inputs) {}

    default void runVolts(double volts) {}

    default void runRadiansPerSecond(double radiansPerSecond) {}

    default void stop() {}
}
