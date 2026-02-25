package org.frc5902.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public boolean connected = false;
        public double[] timestamps = new double[] {};
        public double[][] frames = new double[][] {};
        public long fps = 0;
    }

    default void updateInputs(VisionIOInputs inputs) {}

}
