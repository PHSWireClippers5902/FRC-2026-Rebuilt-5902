package org.frc5902.robot.subsystems.kitbot.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public boolean intakeConnected = false;
        public boolean feederConnected = false;
        public double intakePercentOutput = 0;
        public double feederPercentOutput = 0;
    }
    // loggable inputs
    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setIntakePercentageOutput(double percent) {}

    public default void setFeederPercentageOutput(double percent) {}
}
