package org.frc5902.robot.subsystems.compbot.launcher.flywheel;


import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    class FlywheelIOInputs {
      public FlywheelIOData data = new FlywheelIOData(false, 0.0, 0.0, 0.0, 0.0);
    }
    
    record FlywheelIOData(
        boolean motorConnected,
        double positionRads,
        double velocityRadsPerSec,
        double appliedVoltage,
        double tempCelsius) {}

    default void updateInputs(FlywheelIOInputs inputs) {}

    default void runVolts(double volts) {}

    default void stop() {}

}