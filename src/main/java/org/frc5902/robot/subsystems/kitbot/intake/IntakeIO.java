package org.frc5902.robot.subsystems.kitbot.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public boolean connected = false;
        public double intakePercentOutput = 0;
    }
    // loggable inputs
    public default void updateInputs(IntakeIOInputs inputs) {}
}
