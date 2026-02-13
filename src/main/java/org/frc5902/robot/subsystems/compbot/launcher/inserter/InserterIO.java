package org.frc5902.robot.subsystems.compbot.launcher.inserter;
import org.littletonrobotics.junction.AutoLog;

public interface InserterIO {
    @AutoLog
    class InserterIOInputs {
      public InserterIOData data = new InserterIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    record InserterIOData(
        boolean motorConnected,
        double positionRads,
        double velocityRadsPerSec,
        double appliedVoltage,
        double torqueCurrentAmps,
        double supplyVolts,
        double tempCelsius) {}

    default void updateInputs(InserterIOInputs inputs) {}

    default void runTorqueCurrent(double current) {}

    default void runVolts(double volts) {}

    default void stop() {}

    default void coast() {}
}