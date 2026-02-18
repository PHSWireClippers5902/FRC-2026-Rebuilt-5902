package org.frc5902.robot.subsystems.compbot.intake.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public IntakeIOData data = new IntakeIOData(false, 0.0, 0.0, 0.0, 0.0);
    }

    record IntakeIOData(
            boolean motorConnected,
            double positionRads,
            double velocityRadsPerSec,
            double appliedVoltage,
            double tempCelsius) {}

    default void updateInputs(IntakeIOInputs inputs) {}

    default void runVolts(double volts) {}

    default void runRadiansPerSecond(double radiansPerSecond) {}

    default void stop() {}
}
