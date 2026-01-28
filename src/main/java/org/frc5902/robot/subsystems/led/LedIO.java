package org.frc5902.robot.subsystems.led;

import org.littletonrobotics.junction.AutoLog;

public interface LedIO {

    @AutoLog
    public static class LedIOInputs {
        public boolean connected = false;
    }

    // loggable inputs
    public default void updateInputs(LedIOInputs inputs) {}
    // sets color to a color
    public default void setColor(double colorIndex) {}
}
