package org.frc5902.robot.subsystems.SLAMtake.slam;

import org.littletonrobotics.junction.AutoLog;

public interface SlamIO {
    @AutoLog
    class SlamIOInputs {
        public SlamIOData data = new SlamIOData(false, false, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    record SlamIOData(
            boolean motorConnected,
            boolean limitSwitchTriggered,
            double positionRotations,
            double velocityRadsPerSec,
            double appliedVoltage,
            double tempCelsius,
            double current) {}

    default void resetEncoderToPosition(double position) {}

    default void updateInputs(SlamIOInputs inputs) {}

    default void runVolts(double volts) {}

    default void runRotationsPerSecond(double runRotationsPerSecond) {}

    default void stop() {}

    default void runAngle(double rotations) {}
}
